/*
 *
 *
 * References:
 * [1] How to use a KdTree to search
 * https://pcl.readthedocs.io/projects/tutorials/en/master/kdtree_search.html
 * [2] Fast triangulation of unordered point clouds
 * https://pcl.readthedocs.io/projects/tutorials/en/master/greedy_projection.html
 * [3] Smoothing and normal estimation based on polynomial reconstruction
 * https://pcl.readthedocs.io/projects/tutorials/en/master/resampling.html
 * [4] 移动最小二乘法在点云平滑和重采样中的应用
 * https://www.cnblogs.com/zzk0/p/10468502.html
 * [5] Removing outliers using a StatisticalOutlierRemoval filter
 * https://pcl.readthedocs.io/projects/tutorials/en/master/statistical_outlier.html
 *
 * search 文件夹下和 kdtree 文件夹中都有 kdtree，search 文件夹中重载的查询函数更多一
 * 些。函数基本功能相同，在 kdtree 文件夹下还存在 kdtreeFLANN，不论是使用 search 文件
 * 夹还是 kdtree 文件夹中的方法进行查询时，最终都会去调用 kdtreeFLANN。
 * -------------------------------------------------------------------------- */

#include "point_cloud_handler.h"
#include "util.h"
#include <pcl/surface/mls.h> // For MovingLeastSquares
#include <pcl/surface/gp3.h> // For GreedyProjectionTriangulation
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/io.h>

#define PRINT_INFO 0
#if PRINT_INFO
#define PRINT(fmt, ...) \
    printf("PointCloudHandler: " fmt, ##__VA_ARGS__)
#else
#define PRINT(fmt, ...)
#endif


PointCloudHandler::PointCloudHandler(
        const std::shared_ptr<mmath::CameraProjector> cam_proj,
        mmath::cam::ID cam_id)
    : _cam_proj(cam_proj)
    , _cam_id(cam_id)
    , _kd_tree(std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>())
    , _texture(cv::Mat())
{
}


PointCloudHandler::~PointCloudHandler()
{
}


void PointCloudHandler::bindPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    _coords = point_cloud;
    PRINT("Initial point cloud size: %zu\n", _coords->size());
    updateKDTreeData();

    // Reset point normal and texture
    _texture = cv::Mat();
}


void PointCloudHandler::bindPointCloud(const PointCloudXYZ &point_cloud)
{
    _coords->resize(point_cloud.rows());
    _coords->getMatrixXfMap() = point_cloud.transpose();
    PRINT("Initial point cloud size: %zu\n", _coords->size());
    updateKDTreeData();

    // Reset point normal and texture
    _texture = cv::Mat();
}


void PointCloudHandler::bindTextureAndNormals(
        const cv::Mat &texture, const cv::Mat &normals)
{
    _texture = texture;
    _normals = normals;
}


void PointCloudHandler::voxelDownSampling(float voxel_size)
{
    voxelDownSampling(voxel_size, _coords);
    PRINT("Point cloud size after voxel downsampled: %zu\n", _coords->size());
    updateKDTreeData();
}


struct PointWithOccurrences {
    PointWithOccurrences() : p({0, 0, 0}), n(0) {}
    Eigen::Vector3f p;  // The position of the point
    size_t          n;  // The number of occurrences
};
void PointCloudHandler::voxelDownSampling(
        float voxel_size, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    auto _point_cloud = _coords;

    Eigen::Vector3f max_xyzf(0, 0, 0);
    Eigen::Vector3f min_xyzf(1000, 1000, 1000);
    for(size_t i = 0; i < _point_cloud->size(); i++) {
        const pcl::PointXYZ& pt = _point_cloud->at(i);
        if(max_xyzf[0] < pt.x) max_xyzf[0] = pt.x;
        if(max_xyzf[1] < pt.y) max_xyzf[1] = pt.y;
        if(max_xyzf[2] < pt.z) max_xyzf[2] = pt.z;
        if(min_xyzf[0] > pt.x) min_xyzf[0] = pt.x;
        if(min_xyzf[1] > pt.y) min_xyzf[1] = pt.y;
        if(min_xyzf[2] > pt.z) min_xyzf[2] = pt.z;
    }
    PRINT("x_range:[%f, %f]\n", min_xyzf[0], max_xyzf[0]);
    PRINT("y_range:[%f, %f]\n", min_xyzf[1], max_xyzf[1]);
    PRINT("z_range:[%f, %f]\n", min_xyzf[2], max_xyzf[2]);

    size_t x_node_num = floor((max_xyzf[0] - min_xyzf[0]) / voxel_size) + 1;
    size_t y_node_num = floor((max_xyzf[1] - min_xyzf[1]) / voxel_size) + 1;
    size_t z_node_num = floor((max_xyzf[2] - min_xyzf[2]) / voxel_size) + 1;
    PRINT("voxel size: %zux%zux%zu\n", x_node_num, y_node_num, z_node_num);

    // Group vertices into voxels
    std::vector<std::vector<std::vector<PointWithOccurrences>>> voxels(
                x_node_num, std::vector<std::vector<PointWithOccurrences>>(
                    y_node_num, std::vector<PointWithOccurrences>(
                        z_node_num, PointWithOccurrences())));
    for(size_t i = 0; i < _point_cloud->size(); i++) {
        const pcl::PointXYZ& pt = _point_cloud->at(i);
        int x_idx = floor((pt.x - min_xyzf[0]) / voxel_size);
        int y_idx = floor((pt.y - min_xyzf[1]) / voxel_size);
        int z_idx = floor((pt.z - min_xyzf[2]) / voxel_size);

        //printf("index: %dx%dx%d\n", x_idx, y_idx, z_idx);

        voxels[x_idx][y_idx][z_idx].p += Eigen::Vector3f(pt.x, pt.y, pt.z);
        voxels[x_idx][y_idx][z_idx].n++;
    }

    // Filter out the valid voxels
    point_cloud->clear();
    for(size_t x_idx = 0; x_idx < x_node_num; x_idx++) {
        for(size_t y_idx = 0; y_idx < y_node_num; y_idx++) {
            for(size_t z_idx = 0; z_idx < z_node_num; z_idx++) {
                PointWithOccurrences& pt = voxels[x_idx][y_idx][z_idx];
                if(pt.n > 0) {
                    pt.p /= pt.n;
                    point_cloud->push_back({pt.p[0], pt.p[1], pt.p[2]});
                }
            }
        }
    }
}


void PointCloudHandler::rmOutliersByRadius(float radius, int min_neighbor_num)
{
    std::vector<size_t> indices_to_be_errased;
    for(size_t i = 0; i < _coords->size(); i++){
        const pcl::PointXYZ& query_point = _coords->at(i);

        std::vector<int> indices;
        std::vector<float> squared_distances;
        _kd_tree->radiusSearch(query_point, radius, indices, squared_distances);
        // Find outliers
        if((int)indices.size() < min_neighbor_num) {
            indices_to_be_errased.push_back(i);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::iterator it = _coords->begin();
    size_t count = 0;
    for(size_t idx : indices_to_be_errased) {
        _coords->erase(it + idx - count);
        count++;
    }
    PRINT("Point size after radius-filter: %zu\n", _coords->size());
    updateKDTreeData();
}


void PointCloudHandler::rmOutliersByKNeighbors(int k, float max_neighbor_dis)
{
    std::vector<size_t> indices_to_be_errased;
    for(size_t i = 0; i < _coords->size(); i++){
        const pcl::PointXYZ& query_point = _coords->at(i);

        std::vector<int> indices(k);
        std::vector<float> squared_distances(k);
        _kd_tree->nearestKSearch(query_point, k, indices, squared_distances);

        // Find outliers
        for (int j = 0; j < k; j++) {
            float dis = squared_distances[j];
            if(dis >= max_neighbor_dis*max_neighbor_dis) {
                indices_to_be_errased.push_back(i);
                break;
            }
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::iterator it = _coords->begin();
    size_t count = 0;
    for(size_t idx : indices_to_be_errased) {
        _coords->erase(it + idx - count);
        count++;
    }
    PRINT("Point size after k-neighbors-filter: %zu\n", _coords->size());
    updateKDTreeData();
}


void PointCloudHandler::statisticalOutliersRemoval(size_t k, float std_thresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(_coords);
    sor.setMeanK(k);
    sor.setStddevMulThresh(std_thresh);
    sor.filter(*_coords);

    PRINT("Point size after statistical-filter: %zu\n", _coords->size());
    updateKDTreeData();
}


void PointCloudHandler::createMesh(
        float search_radius, float mu, int max_neighbors,
        std::vector<mlayer::Vertex3D>& vertices)
{
    if(_texture.empty()){
        PRINT("No texture was binded!\n");
        std::abort();
    }

    // MSL for smoothing
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (
                new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(_coords);

    pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_with_normal(
                new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(_coords);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(search_radius);
    mls.process(*point_cloud_with_normal);
    PRINT("Point size after mls: %zu\n", point_cloud_with_normal->size());


    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
                new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(point_cloud_with_normal);

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
    gp3.setInputCloud(point_cloud_with_normal);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(mesh);

    // Assign mesh vertices
    float r, g, b;
    size_t count = 0;
    vertices.resize(mesh.polygons.size() * 3);
    for(size_t i = 0; i < mesh.polygons.size(); i++) {
        pcl::Vertices vert = mesh.polygons[i];

        for(size_t j = 0; j < vert.vertices.size(); j++) {
            size_t idx = vert.vertices[j];
            auto& pt = _coords->at(idx);
            readPointColor(pt, r, g, b);
            vertices[count++] = {
                glm::vec4(pt.x, pt.y, pt.z, 1), glm::vec4(r, g, b, 1)};
        }
    }
    PRINT("Create mesh (size of [%zu]) with [%zu] vertices.\n",
          mesh.polygons.size(), count);
}


void PointCloudHandler::createVertices(std::vector<mlayer::Vertex3D> &vertices)
{
    if(_texture.empty()){
        PRINT("No texture was binded!\n");
        std::abort();
    }

    float r, g, b;
    vertices.resize(_coords->size());
    for(size_t i = 0; i < _coords->size(); i++) {
        auto& pt = _coords->at(i);
        readPointColor(pt, r, g, b);
        vertices[i] = {glm::vec4(pt.x, pt.y, pt.z, 1), glm::vec4(r, g, b, 1)};
    }
}


const pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudHandler::getCurrentPointCloud() const
{
    return _coords;
}


pcl::PointCloud<pcl::PointNormal>::Ptr
PointCloudHandler::getCurrentPointCloudWithNormal() const
{
    pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_with_normal
            = util::estimateNormal(_coords);
    return point_cloud_with_normal;
}


Eigen::MatrixXf PointCloudHandler::toEigenMatrix() const
{
    Eigen::MatrixXf mat(_coords->size(), 3);
    mat = _coords->getMatrixXfMap().transpose();
    return mat;
}


Vertices PointCloudHandler::createVertices()
{
    if(_texture.empty() || _normals.empty()){
        PRINT("No texture and normals was binded!\n");
        std::abort();
    }

    Vertices vertices;
    vertices.resize(_coords->size());
#if 0
    // MSL for smoothing
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (
                new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(_coords);

    pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_with_normal(
                new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(_coords);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(3);
    mls.process(*point_cloud_with_normal);

    float r, g, b;
    for(size_t i = 0; i < _coords->size(); i++) {
        vertices[i].coord = point_cloud_with_normal->at(i).getVector3fMap();
        vertices[i].normal = point_cloud_with_normal->at(i).getNormalVector3fMap();

        auto& pt = vertices[i].pclCoord();
        readPointColor(pt, r, g, b);
        vertices[i].color = {r, g, b};
        vertices[i].weight = 1;
    }
#else
    float r, g, b;
    for(size_t i = 0; i < _coords->size(); i++) {
        auto& pt = _coords->at(i);
        Eigen::Vector2f pt2d = _cam_proj->cvt3Dto2D(pt.x, pt.y, pt.z, _cam_id);
        int u = round(pt2d[0]);
        int v = round(pt2d[1]);
        const cv::Vec3b& pixel = _texture.at<cv::Vec3b>(v, u);
        r = 1.f*pixel[0]/255.f;
        g = 1.f*pixel[1]/255.f;
        b = 1.f*pixel[2]/255.f;
        cv::Vec3f cvnormal = _normals.at<cv::Vec3f>(v, u);

        vertices[i].coord = pt.getVector3fMap();
        vertices[i].normal = {cvnormal[0], cvnormal[1], cvnormal[2]};
        vertices[i].color = {r, g, b};
        vertices[i].weight = 1;
    }
#endif
    // The last two terms in the vertex will be initialized in fusion step in
    // ED procudure.
    return vertices;
}


cv::Mat PointCloudHandler::projectVerticesToDepthmap() const
{
    cv::Mat depthmap(_texture.size(), CV_32FC1, cv::Scalar(0));
    for(size_t i = 0; i < _coords->size(); i++) {
        const Eigen::Vector3f& pt = _coords->at(i).getVector3fMap();
        Eigen::Vector2f pt2d = _cam_proj->cvt3Dto2D(pt, _cam_id);
        int u = round(pt2d[0]);
        int v = round(pt2d[1]);
        depthmap.at<float>(v, u) = pt[2];
    }

    return depthmap;
}


void PointCloudHandler::updateKDTreeData()
{
    _kd_tree->setInputCloud(_coords);
}


void PointCloudHandler::readPointColor(
        const pcl::PointXYZ& pt, float &r, float &g, float &b)
{
    Eigen::Vector2f pt2d = _cam_proj->cvt3Dto2D(
                pt.x, pt.y, pt.z, _cam_id);
    int u = round(pt2d[0]);
    int v = round(pt2d[1]);
    const cv::Vec3b& pixel = _texture.at<cv::Vec3b>(v, u);
    r = 1.f*pixel[0]/255.f;
    g = 1.f*pixel[1]/255.f;
    b = 1.f*pixel[2]/255.f;
}


/* -------------------------------------------------------------------------- */
/*                            namespace::util                                 */
/* -------------------------------------------------------------------------- */

namespace util {

void vectorPoints2pclPoints(const std::vector<Eigen::Vector3f>& in,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    out->resize(in.size());
    for(size_t i = 0; i < in.size(); i++) {
        out->at(i) = pcl::PointXYZ(in[i][0], in[i][1], in[i][2]);
    }
}


void eigenPoints2pclPoints(const PointCloudXYZ& in,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    out->resize(in.rows());
    for(long i = 0; i < in.rows(); i++) {
        out->at(i) = pcl::PointXYZ(in(i, 0), in(i, 1), in(i, 2));
    }
}

} // namespace::util
