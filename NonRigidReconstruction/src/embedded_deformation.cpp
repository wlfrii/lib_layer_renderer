#include "embedded_deformation.h"
#include <vector>
#include "util.h"
#include "cost_function.h"
#include <pcl/surface/gp3.h> // For GreedyProjectionTriangulation
#include "logger.h"

const float w_rot = 1000000.f;
const float w_reg = 10000.f;
const float w_data = 1000.f;
const float w_corr = 10.f;
const int max_LM_iter = 15; // experimentally


EmbeddedDeformation::EmbeddedDeformation(
        const std::shared_ptr<mmath::CameraProjector> cam_proj,
        const mmath::cam::ID cam_id,
        float vertex_density,
        float node_density, int node_connectivity)
    : _cam_proj(cam_proj), _cam_id(cam_id)
    , _vertex_density(vertex_density)
    , _node_density(node_density)
    , _node_connectivity(node_connectivity)
    , _K(_node_connectivity + 1)
    , _nodes(new pcl::PointCloud<pcl::PointXYZ>)
    , _kd_tree(new pcl::KdTreeFLANN<pcl::PointXYZ>)
//    , _R(Eigen::Matrix3f::Identity())
//    , _t(Eigen::Vector3f::Zero())
    , _timestamp(0)
{
    // The number of neighbor nodes
    int k = std::ceil(_node_connectivity / 4.f);
    _neighbor_dis_thresh = std::powf(sqrt(2), k) * _node_density;

    printf("Construct an EmbeddedDeformation object. Node density is set to "
           "%.1f, node connectivity is set to %d, and max neighbor distance is"
           "determined to %f\n",
           _node_density, _node_connectivity, _neighbor_dis_thresh);
}


EmbeddedDeformation::~EmbeddedDeformation()
{

}


void EmbeddedDeformation::addVertices(const Vertices& new_vertices)
{
    _timestamp++;
    _vertices = new_vertices;
    for(Vertex& vert : _vertices) {
        vert.timestamp = _timestamp;
    }
    regenerateNodes();
}


void EmbeddedDeformation::addVertices(
        const Vertices& new_vertices,
        const cv::Mat &depthmap, const cv::Mat& texture,
        const std::vector<
            std::pair<pcl::PointXYZ, pcl::PointXYZ>>& correspondences)
{
    _timestamp++;

    if(correspondences.size() < 5) return;

    // Transform the vertices w.r.t current camera pose
    Eigen::Matrix3f global_R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f global_t = Eigen::Vector3f::Zero();
    util::estimateTransform(correspondences, global_R, global_t);

    mmath::Pose pose(global_R, global_t);
    std::cout << "Correspondences transform:\n" << pose << std::endl;


    // Register the visible point
    Vertices visible_model_points;
    depthImageRegistration(global_R, global_t, depthmap, visible_model_points);

    // Initilize the parameters that needs to be optimized
    std::vector<double> init_Rt = { 1,0,0, 0,1,0, 0,0,1, 0,0,0}; // [x, y, z, t]
    std::vector<std::vector<double>> params(
                _nodes->size(), init_Rt);

    /* Build levenberg marquardt optimization */
    WLF_VERBOSE("Building problem ......\n");
    ceres::Problem problem;

    // Add rotation term
    for(size_t i = 0; i < _nodes->size(); i++) {
        CostFunctionRot* cost_func = new CostFunctionRot(w_rot);
        problem.AddResidualBlock(cost_func, nullptr, &params[i][0]);
    }
    // Add regularization term
    for(size_t i = 0; i < _nodes->size(); i++) {
        const pcl::PointXYZ& node = _nodes->at(i);
        Eigen::Vector3d g_j = node.getVector3fMap().cast<double>();

        std::vector<int> indices(_K);
        std::vector<float> squared_distances(_K);
        _kd_tree->nearestKSearch(node, _K, indices, squared_distances);
        for(size_t idx : indices) {
            if(idx == i) continue;

            const pcl::PointXYZ& node_k = _nodes->at(idx);
            Eigen::Vector3d g_k = node_k.getVector3fMap().cast<double>();
            CostFunctionReg* cost_func = new CostFunctionReg(w_reg, g_j, g_k);
            problem.AddResidualBlock(cost_func, nullptr, &params[i][0], &params[idx][0]);
        }
    }
    /* Add data term, which is the sum of point-to-plane errors
     * After extracting registered visible points, back-projection approach is
     * adopted as a model-to-scan registration strategy that penalizes
     * misalignment of the predicted visible points
     */
    size_t count_invalid_data = 0;
    for(size_t i = 0; i < visible_model_points.size(); i++){
        const Vertex& vert = visible_model_points[i];
        Eigen::Vector3d visible_point = vert.coord.cast<double>();
        Eigen::Vector3d visible_point_normal = vert.normal.cast<double>();

        // Find the neighor nodes of each old vertices
        const pcl::PointXYZ& src_pt = vert.pclCoord();
        std::vector<int> indices(_K);
        std::vector<float> squared_distances(_K);
        _kd_tree->nearestKSearch(src_pt, _K, indices, squared_distances);

        if(sqrtf(squared_distances.back()) > _neighbor_dis_thresh) {
//            printf("Reject data term. Max squared distance = %f\n",
//                   squared_distances.back());
            count_invalid_data++;
            continue;
        }

        Eigen::VectorXd w_j = util::nodeWeights(squared_distances);

//        std::cout << "Visible point: " << visible_point.transpose()
//                  << "\tVisible point normal: " << visible_point_normal.transpose()
//                  << "\n";
//        std::cout << "\t Wj: " << w_j.transpose()
//                  << "\n";

        std::vector<Eigen::Vector3d> neighbor_nodes(_K);
        std::vector<double*> parameter_blocks(_K);
        for(int j = 0; j < _K; j++) {
            int idx = indices[j];
            neighbor_nodes[j] = _nodes->at(idx).getVector3fMap().cast<double>();
            parameter_blocks[j] = (&params[idx][0]);
        }

        ceres::CostFunction* cost_func = CostFunctionData::create(
                    w_data, visible_point, visible_point_normal,
                    neighbor_nodes, w_j, _cam_proj->fxy, _cam_proj->cx,
                    _cam_proj->cy, _cam_proj->t, depthmap,
                    global_R.cast<double>(), global_t.cast<double>());
        problem.AddResidualBlock(cost_func, nullptr, parameter_blocks);
    }
    WLF_VERBOSE("Invalid data term size: %zu/%zu\n",
                count_invalid_data, visible_model_points.size());
    // Add correspondence term
    size_t count_invalid_corr = 0;
    for(auto& corr : correspondences) {
        Eigen::Vector3d v_prev = corr.first.getVector3fMap().cast<double>();
        Eigen::Vector3d v_curr = corr.second.getVector3fMap().cast<double>();

        // Find the neighor nodes of each old vertices
        const pcl::PointXYZ& src_pt = corr.first;
        std::vector<int> indices(_K);
        std::vector<float> squared_distances(_K);
        _kd_tree->nearestKSearch(src_pt, _K, indices, squared_distances);


        if(sqrtf(squared_distances.back()) > _neighbor_dis_thresh) {
//            printf("Reject corr term. Max squared distance = %f\n",
//                   squared_distances.back());
            count_invalid_corr++;
            continue;
        }

        Eigen::VectorXd w_j = util::nodeWeights(squared_distances);

        std::vector<Eigen::Vector3d> neighbor_nodes(_K);
        std::vector<double*> parameter_blocks(_K);
        for(int j = 0; j < _K; j++) {
            int idx = indices[j];
            neighbor_nodes[j] = _nodes->at(idx).getVector3fMap().cast<double>();
            parameter_blocks[j] = (&params[idx][0]);
        }

        CostFunctionCorr* cost_func = new CostFunctionCorr(
                    w_corr, v_prev, neighbor_nodes, w_j, v_curr,
                    global_R.cast<double>(), global_t.cast<double>());
        problem.AddResidualBlock(cost_func, nullptr, parameter_blocks);
    }
    WLF_VERBOSE("Invalid correspondence size: %zu/%zu\n",
                count_invalid_corr, correspondences.size());

    ceres::Solver::Options options;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 0.01;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;
    options.max_num_iterations = max_LM_iter;
    options.function_tolerance = 1e-10;
    options.parameter_tolerance = 1e-10;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    WLF_VERBOSE("Solve problem done!\n");

    // Obtain the estimated nodes Rt
    std::vector<Eigen::Matrix3d> Rs(_nodes->size());
    std::vector<Eigen::Vector3d> ts(_nodes->size());
    for (size_t i = 0; i < _nodes->size(); ++i)
    {
        Rs[i] = Eigen::Map<Eigen::Matrix3d>(&(params[i][0]));
        ts[i] = Eigen::Map<Eigen::Vector3d>(&(params[i][0]) + 9);
    }

    // Deform the vertices
    for(Vertex& vert : _vertices) {
        const pcl::PointXYZ& pt = vert.pclCoord();
        Eigen::Vector3d vi = vert.coord.cast<double>();
        Eigen::Vector3d ni = vert.normal.cast<double>();

        std::vector<int> indices(_K);
        std::vector<float> squared_distances(_K);
        _kd_tree->nearestKSearch(pt, _K, indices, squared_distances);
        Eigen::VectorXd w_j = util::nodeWeights(squared_distances);

        Eigen::Vector3d new_v(0, 0, 0);
        Eigen::Vector3d new_n(0, 0, 0);
        for(int j = 0; j < _K; j++) {
            int jj = indices[j];
            Eigen::Vector3d gj = _nodes->at(jj).getVector3fMap().cast<double>();
            new_v += w_j[j] * (Rs[jj]*(vi - gj) + gj + ts[jj]);
            new_n += w_j[j] * Rs[jj] * ni;
        }
        new_n.normalize();

        new_v = global_R.cast<double>() * new_v + global_t.cast<double>();
        new_n = global_R.cast<double>() * new_n;

        vert.coord = new_v.cast<float>();
        vert.normal = new_n.cast<float>();

//        if(i % 2000 == 0){
//            printf("[%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f] \t"
//                   "[%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f] \n",
//                   vi[0], vi[1], vi[2], new_v[0], new_v[1], new_v[2],
//                   ni[0], ni[1], ni[2], new_n[0], new_n[1], new_n[2]);
//        }
    }
    WLF_VERBOSE("Deform current vertices done!\n");

    // Fusion the deformated coords with new observation
    // Test for directly fusion all the points
    fusionVertices(new_vertices, depthmap, texture);


    // Regenerate nodes
    regenerateNodes();
}


const Vertices& EmbeddedDeformation::getVertices() const
{
    return _vertices;
}


void EmbeddedDeformation::projectPointCloud(
        float search_radius, float mu, int max_neighbors,
        std::vector<mlayer::Vertex3D> &vertices)
{
    auto pclpoints_with_normals = _vertices.pclPointCloudWithNormals();

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
                new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(pclpoints_with_normals);

    // ---- Create triangles ---
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setSearchRadius(search_radius);    // max edge length for every triangle
    gp3.setMu(mu); // maximum acceptable distance for a point to be considered as a neighbor
    gp3.setMaximumNearestNeighbors(max_neighbors);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18.f); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    pcl::PolygonMesh mesh;
    gp3.setInputCloud(pclpoints_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(mesh);

    // Assign mesh vertices
    size_t count = 0;
    vertices.resize(mesh.polygons.size() * 3);
    for(size_t i = 0; i < mesh.polygons.size(); i++) {
        pcl::Vertices vert = mesh.polygons[i];

        for(size_t j = 0; j < vert.vertices.size(); j++) {
            size_t idx = vert.vertices[j];

            const Eigen::Vector3f& pt = _vertices[idx].coord;
            const Eigen::Vector3f& rgb = _vertices[idx].color;
            vertices[count++] = {
                glm::vec4(pt[0], pt[1], pt[2], 1),
                glm::vec4(rgb[0], rgb[1], rgb[2], 1)};
        }
    }
}


void EmbeddedDeformation::projectPointCloud(
        std::vector<mlayer::Vertex3D> &vertices)
{
    // Assign mesh vertices
    size_t count = 0;
    vertices.resize(_vertices.size() * 3);
    for(size_t i = 0; i < _vertices.size(); i++) {
        const Vertex vert = _vertices[i];

        const Eigen::Vector3f& pt = vert.coord;
        const Eigen::Vector3f& rgb = vert.color;
        vertices[count++] = {
            glm::vec4(pt[0], pt[1], pt[2], 1),
            glm::vec4(rgb[0], rgb[1], rgb[2], 1)};
    }
}


void EmbeddedDeformation::regenerateNodes()
{
    util::voxelDownSampling(_vertices, _node_density, _nodes);
    WLF_VERBOSE("Nodes size: %zu\n", _nodes->size());
    _kd_tree->setInputCloud(_nodes);
}


void EmbeddedDeformation::depthImageRegistration(
        const Eigen::Matrix3f& R, const Eigen::Vector3f& t,
        const cv::Mat& depthmap, Vertices &visible_points)
{
    visible_points.clear();

    float epsilon_d = 1.2 * VERTEX_DENSITY;    // mm
    float epsilon_n = mmath::deg2radf(10);
//    printf("Visible points thresholds: epsilon_p:%f, epsilon_n:%f\n",
//           epsilon_d, cosf(epsilon_n));

    // Calcualte the pixel normal
    cv::Mat normal_image;
    util::estimateImageNormal(depthmap, normal_image);

    for(size_t i = 0; i < _vertices.size(); i++) {
        const Vertex& vert = _vertices[i];

        // Transform vertex to current camera frame
        Eigen::Vector3f coord = R * vert.coord + t;

        Eigen::Vector2f pt2d = _cam_proj->cvt3Dto2D(coord, _cam_id);
        int u = round(pt2d[0]);
        int v = round(pt2d[1]);
        if(u < 0 || u >= depthmap.cols || v < 0 || v >= depthmap.rows){
            continue;
        }

        float d = depthmap.at<float>(v, u);
        if(d >= 30 && d <= 150) {
            Eigen::Vector3f new_coord = _cam_proj->cvt2Dto3D(
                        u, v, d, _cam_id);
            cv::Vec3f cvnormal = normal_image.at<cv::Vec3f>(v, u);
            Eigen::Vector3f new_normal(cvnormal[0], cvnormal[1], cvnormal[2]);

            if((vert.coord - new_coord).norm() < epsilon_d &&
                    (vert.normal.dot(new_normal) < cosf(epsilon_n)) ) {
                visible_points.push_back(vert);
            }
        }
    }
    // Model points is stored in ED
    WLF_VERBOSE("Visible point size: %zu/%zu\n",
                visible_points.size(), _vertices.size());
}


void EmbeddedDeformation::fusionVertices(const Vertices& new_vertices,
                                         const cv::Mat& depthmap,
                                         const cv::Mat& texture)
{
    // Now, the model vertices is deformed
    float epsilon_d = 1.2 * _vertex_density;    // mm
    float epsilon_n = mmath::deg2radf(10);
//    printf("Visible points thresholds: epsilon_p:%f, epsilon_n:%f\n",
//           epsilon_d, cosf(epsilon_n));

    // Calcualte the pixel normal
    cv::Mat normal_image;
    util::estimateImageNormal(depthmap, normal_image);

    for(Vertex& vert : _vertices) {
        Eigen::Vector2f pt2d = _cam_proj->cvt3Dto2D(vert.coord, _cam_id);
        int u = round(pt2d[0]);
        int v = round(pt2d[1]);
        float d = depthmap.at<float>(v, u);
        if(u < 0 || u >= depthmap.cols || v < 0 || v >= depthmap.rows){
            continue;
        }

        if(d >= 30 && d <= 150) {
            cv::Vec3b rgb = texture.at<cv::Vec3b>(v, u);
            Eigen::Vector3f new_color(1.f*rgb[0]/255, 1.f*rgb[1]/255, 1.f*rgb[2]/255);

            Eigen::Vector3f new_coord = _cam_proj->cvt2Dto3D(
                        u, v, d, _cam_id);
            cv::Vec3f cvnormal = normal_image.at<cv::Vec3f>(v, u);
            Eigen::Vector3f new_normal(cvnormal[0], cvnormal[1], cvnormal[2]);

            if((vert.coord - new_coord).norm() < epsilon_d &&
                    (vert.normal.dot(new_normal) < cosf(epsilon_n)) ) {
                float w = vert.weight + 1;

                vert.coord[2] = (vert.coord[2] * vert.weight + d) / w;
                vert.normal = (vert.normal * vert.weight + new_normal) / w;
                vert.normal.normalize();
                vert.color = (vert.color * vert.weight + new_color) / w;
                vert.weight = fminf(vert.weight + 1,VERTEX_MAX_WEIGHT);
            }
        }
    }

    // Fusion new observation
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
    kd_tree.setInputCloud(_vertices.pclPointCloud());

    size_t count = 0;
    for(Vertex vert : new_vertices) {
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        kd_tree.nearestKSearch(vert.pclCoord(), 1,
                               indices, squared_distances);
        if(sqrt(squared_distances[0]) >= epsilon_d) {
            vert.weight = 1;
            vert.timestamp = _timestamp;
            vert.stability = false;
            _vertices.emplace_back(vert);
            count++;
        }
    }
    WLF_VERBOSE("New added vertices size: %zu\n", count);
    WLF_VERBOSE("Point size after fusion: %zu\n", _vertices.size());


//    if(_timestamp > 1) {
//        for(size_t i = 0; i < _vertices.size(); i+=300) {
//            Vertex& vert = _vertices[i];
//            printf("%06zu  _timestamp:%zu,  vert.t: %zu | vert.weight: %f | "
//                   "vert.stablility: %d\n", i, _timestamp, vert.timestamp,
//                   vert.weight, vert.stability);
//        }
//        cv::waitKey(0);
//    }


    // Filter out unstable points (refers to MIS-SLAM)

    Vertices temp;
    temp.resize(_vertices.size());
    count = 0;
    for(size_t i = 0; i < _vertices.size(); i++) {
        Vertex& vert = _vertices[i];

        if((1.f*_timestamp - vert.timestamp) > TAO_TIME &&
                vert.weight < TAO_WEIGHT &&
                vert.stability == false){
            // delete this vert
            continue;
        }
        else{
            if((1.f*_timestamp - vert.timestamp) < TAO_TIME &&
                    vert.weight >= TAO_WEIGHT){
                vert.stability = true;
            }
        }
        temp[count++] = vert;
    }
    WLF_VERBOSE("Point size to be removed: %zu\n", _vertices.size() - count);

    Vertices::iterator it = temp.begin();
    temp.erase(it + count, temp.end());
    _vertices = temp;

    WLF_VERBOSE("Point size after filtering out noise: %zu\n", _vertices.size());
}
