#include "embedded_deformation.h"
#include <vector>
#include "util.h"
#include "cost_function.h"
#include <pcl/surface/gp3.h> // For GreedyProjectionTriangulation

const float w_rot = 1000.f;
const float w_reg = 10000.f;
const float w_data = 1.f;
const float w_corr = 10000.f;


EmbeddedDeformation::EmbeddedDeformation(
        float node_density, int node_connectivity)
    : _node_density(node_density)
    , _node_connectivity(node_connectivity)
    , _nodes(new pcl::PointCloud<pcl::PointXYZ>)
    , _kd_tree(new pcl::KdTreeFLANN<pcl::PointXYZ>)
{
    _vertices.coords = nullptr;
}


EmbeddedDeformation::~EmbeddedDeformation()
{

}


void EmbeddedDeformation::addVertices(const Vertices& new_vertices)
{
    _vertices = new_vertices;
    util::voxelDownSampling(_vertices.coords, _node_density, _nodes);
    printf("Nodes size: %zu\n", _nodes->size());

    _Rs = std::vector<Eigen::Matrix3d>(_nodes->size(),
                                       Eigen::Matrix3d::Identity());
    _ts = std::vector<Eigen::Vector3d>(_nodes->size(),
                                       Eigen::Vector3d::Zero());

    updateKdTreeData();
}


void EmbeddedDeformation::addVertices(
        const Vertices& new_vertices, const cv::Mat& depthmap,
        const std::vector<
            std::pair<pcl::PointXYZ, pcl::PointXYZ>>& correspondences)
{
    for(auto& corr : correspondences){
        printf("prev [%.4f, %.4f, %.4f] - curr [%.4f, %.4f, %.4f]\n",
               corr.first.x, corr.first.y, corr.first.z,
               corr.second.x, corr.second.y, corr.second.z);
    }

    int K = _node_connectivity + 1;

    // Initilize the parameters that needs to be optimized
    std::vector<std::vector<double>> params(
                _nodes->size(), std::vector<double>(12));
    for(size_t i = 0; i < _nodes->size(); i++) {
        params[i][0] = _Rs[i](0, 0);
        params[i][1] = _Rs[i](1, 0);
        params[i][2] = _Rs[i](2, 0);
        params[i][3] = _Rs[i](0, 1);
        params[i][4] = _Rs[i](1, 1);
        params[i][5] = _Rs[i](2, 1);
        params[i][6] = _Rs[i](0, 2);
        params[i][7] = _Rs[i](1, 2);
        params[i][8] = _Rs[i](2, 2);
        params[i][9] = _ts[i][0];
        params[i][10] = _ts[i][1];
        params[i][11] = _ts[i][2];
    }

    /* Build levenberg marquardt optimization */
    printf("Building problem ......\n");
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

        std::vector<int> indices(K);
        std::vector<float> squared_distances(K);
        _kd_tree->nearestKSearch(node, K, indices, squared_distances);
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
    // Registrates current point cloud to the ED model and stores the
    // visible points
    pcl::PointCloud<pcl::PointNormal>::Ptr visible_points(
                new pcl::PointCloud<pcl::PointNormal>);
    depthImageRegistration(depthmap, visible_points);
    for(size_t i = 0; i < visible_points->size(); i++){

    }
    // Add correspondence term
    for(auto& corr : correspondences) {
        Eigen::Vector3d v_prev = corr.first.getVector3fMap().cast<double>();
        Eigen::Vector3d v_curr = corr.second.getVector3fMap().cast<double>();

        // Find the neighor nodes of each old vertices
        const pcl::PointXYZ& src_pt = corr.first;
        std::vector<int> indices(K);
        std::vector<float> squared_distances(K);
        _kd_tree->nearestKSearch(src_pt, K, indices, squared_distances);

        Eigen::VectorXd w_j = util::nodeWeights(squared_distances);

        std::vector<Eigen::Vector3d> neighbor_nodes(K);
        std::vector<double*> parameter_blocks(K);
        for(int j = 0; j < K; j++) {
            int idx = indices[j];
            neighbor_nodes[j] = _nodes->at(idx).getVector3fMap().cast<double>();
            parameter_blocks[j] = (&params[idx][0]);
        }

        CostFunctionCorr* cost_func = new CostFunctionCorr(
                    w_corr, v_prev, neighbor_nodes, w_j, v_curr);
        problem.AddResidualBlock(cost_func, nullptr, parameter_blocks);
    }

    ceres::Solver::Options options;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 0.01;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;
    options.max_num_iterations = 100;
    options.function_tolerance = 1e-10;
    options.parameter_tolerance = 1e-10;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    printf("Solve problem done!\n");

    for(size_t i = 0; i < _nodes->size(); i++) {
        _Rs[i] = Eigen::Map<Eigen::Matrix3d>(&(params[i][0]));
        _ts[i] = Eigen::Map<Eigen::Vector3d>(&(params[i][0]) + 9);
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr deformed_coords(
                new pcl::PointCloud<pcl::PointXYZ>);
    deformed_coords->resize(_vertices.coords->size());

    for(size_t i = 0; i < _vertices.coords->size(); i++) {
        const pcl::PointXYZ& pt = _vertices.coords->at(i);
        Eigen::Vector3d vi = pt.getVector3fMap().cast<double>();

        std::vector<int> indices(K);
        std::vector<float> squared_distances(K);
        _kd_tree->nearestKSearch(pt, K, indices, squared_distances);
        Eigen::VectorXd w_j = util::nodeWeights(squared_distances);

        Eigen::Vector3d new_pt(0, 0, 0);
        for(int j = 0; j < K; j++) {
            int jj = indices[j];
            Eigen::Vector3d gj = _nodes->at(jj).getVector3fMap().cast<double>();
            new_pt += w_j[j] * (_Rs[jj]*(vi - gj) + gj + _ts[jj]);
        }
        deformed_coords->at(i) = pcl::PointXYZ(new_pt[0], new_pt[1], new_pt[2]);

        if(i < 5){
            printf("[%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]\n",
                   pt.x, pt.y, pt.z, new_pt[0], new_pt[1], new_pt[2]);
        }
    }

    _vertices.coords = deformed_coords;
}


const Vertices& EmbeddedDeformation::getVertices() const
{
    return _vertices;
}


void EmbeddedDeformation::projectPointCloud(
        float search_radius, float mu, int max_neighbors,
        std::vector<mlayer::Vertex3D> &vertices)
{
    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
                new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(_vertices.coords_with_normals);

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
    gp3.setInputCloud(_vertices.coords_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(mesh);

    // Assign mesh vertices
    size_t count = 0;
    vertices.resize(mesh.polygons.size() * 3);
    for(size_t i = 0; i < mesh.polygons.size(); i++) {
        pcl::Vertices vert = mesh.polygons[i];

        for(size_t j = 0; j < vert.vertices.size(); j++) {
            size_t idx = vert.vertices[j];

            const pcl::PointXYZ& pt = _vertices.coords->at(idx);
            const Eigen::Vector3f& rgb = _vertices.colors[idx];
            vertices[count++] = {
                glm::vec4(pt.x, pt.y, pt.z, 1),
                glm::vec4(rgb[0], rgb[1], rgb[2], 1)};
        }
    }
}


void EmbeddedDeformation::updateKdTreeData()
{
    _kd_tree->setInputCloud(_nodes);
}


void EmbeddedDeformation::depthImageRegistration(const cv::Mat& depthmap,
        pcl::PointCloud<pcl::PointNormal>::Ptr visible_points)
{
    float epsilon_d = 3.f;    // mm
    float epsilon_n = mmath::deg2radf(10);

    // Get normal for current Depthmap
    cv::Mat normal_image;
    util::estimateImageNormal(depthmap, normal_image);


    // Model points is stored in ED
    const pcl::PointCloud<pcl::PointNormal>::Ptr prev_point_cloud =
            _vertices.coords_with_normals;

    for(size_t i = 0; i < prev_point_cloud->size(); i++) {
        const Eigen::Vector3f& pt = prev_point_cloud->at(i).getVector3fMap();
        const Eigen::Vector3f& n = prev_point_cloud->at(i).getNormalVector3fMap();

        Eigen::Vector2f pt2d = _cam_proj->cvt3Dto2D(
                    pt, mmath::cam::LEFT);
        int u = round(pt2d[0]);
        int v = round(pt2d[1]);

        float d = depthmap.at<float>(v, u);
        if(d > 30) {
            Eigen::Vector3f pt3d = _cam_proj->cvt2Dto3D(
                        u, v, d, mmath::cam::LEFT);
            cv::Vec3f cvnormal = normal_image.at<cv::Vec3f>(v, u);
            Eigen::Vector3f pixel_normal(cvnormal[0], cvnormal[1], cvnormal[2]);

            if((pt - pt3d).norm() < epsilon_d ||
                   (n.dot(pixel_normal) < cosf(epsilon_n)) ) {
                visible_points->push_back(prev_point_cloud->at(i));
            }
        }
    }
    printf("visible point size: %zu\n", visible_points->size());
    printf("total point size: %zu\n", prev_point_cloud->size());
}
