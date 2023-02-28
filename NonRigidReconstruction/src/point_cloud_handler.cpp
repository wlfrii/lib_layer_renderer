/*
 *
 *
 * References:
 * [1] Fast triangulation of unordered point clouds
 * https://pcl.readthedocs.io/projects/tutorials/en/master/greedy_projection.html
 * [2] Smoothing and normal estimation based on polynomial reconstruction
 * https://pcl.readthedocs.io/projects/tutorials/en/master/resampling.html
 * [3] 移动最小二乘法在点云平滑和重采样中的应用
 * https://www.cnblogs.com/zzk0/p/10468502.html
 *
 * search 文件夹下和 kdtree 文件夹中都有 kdtree，search 文件夹中重载的查询函数更多一
 * 些。函数基本功能相同，在 kdtree 文件夹下还存在 kdtreeFLANN，不论是使用 search 文件
 * 夹还是 kdtree 文件夹中的方法进行查询时，最终都会去调用 kdtreeFLANN。
 * -------------------------------------------------------------------------- */

#include "point_cloud_handler.h"
#include "util.h"
#include <pcl/surface/mls.h> // For MovingLeastSquares
#include <pcl/surface/gp3.h> // For GreedyProjectionTriangulation
#include <pcl/search/flann_search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>


#define KDTREE_PTR \
    std::make_shared<nanoflann::KDTreeEigenMatrixAdaptor<PointCloudXYZ>>


PointCloudHandler::PointCloudHandler(const PointCloudXYZ& point_cloud)
    : _point_cloud(point_cloud)
    , _kd_tree(nullptr)
{
    setKDTree();
}


PointCloudHandler::PointCloudHandler(const std::vector<Eigen::Vector3f>& point_cloud)
    : _kd_tree(nullptr)
{
    util::vectorPoints2EigenPoints(point_cloud, _point_cloud);
    setKDTree();
}


PointCloudHandler::~PointCloudHandler()
{
}


struct PointWithOccurrences {
    PointWithOccurrences() : p({0, 0, 0}), n(0) {}
    Eigen::Vector3f p;  // The position of the point
    size_t          n;  // The number of occurrences
};
void PointCloudHandler::voxelDownSampling(float voxel_size)
{
    Eigen::Vector3f max_xyzf = _point_cloud.colwise().maxCoeff();
    Eigen::Vector3f min_xyzf = _point_cloud.colwise().minCoeff();

    printf("x_range:[%f, %f]\n", min_xyzf[0], max_xyzf[0]);
    printf("y_range:[%f, %f]\n", min_xyzf[1], max_xyzf[1]);
    printf("z_range:[%f, %f]\n", min_xyzf[2], max_xyzf[2]);

    size_t x_node_num = floor((max_xyzf[0] - min_xyzf[0]) / voxel_size) + 1;
    size_t y_node_num = floor((max_xyzf[1] - min_xyzf[1]) / voxel_size) + 1;
    size_t z_node_num = floor((max_xyzf[2] - min_xyzf[2]) / voxel_size) + 1;
    printf("voxel size: %zux%zux%zu\n", x_node_num, y_node_num, z_node_num);

    // Group vertices into voxels
    std::vector<std::vector<std::vector<PointWithOccurrences>>> voxels(
                x_node_num, std::vector<std::vector<PointWithOccurrences>>(
                    y_node_num, std::vector<PointWithOccurrences>(
                        z_node_num, PointWithOccurrences())));
    for(int i = 0; i < _point_cloud.rows(); i++) {
        int x_idx = floor(_point_cloud(i, 0) - min_xyzf[0] / voxel_size);
        int y_idx = floor(_point_cloud(i, 1) - min_xyzf[1] / voxel_size);
        int z_idx = floor(_point_cloud(i, 2) - min_xyzf[2] / voxel_size);

        //printf("index: %dx%dx%d\n", x_idx, y_idx, z_idx);

        voxels[x_idx][y_idx][z_idx].p += _point_cloud.row(i);
        voxels[x_idx][y_idx][z_idx].n++;
    }

    // Filter out the valid voxels
    std::vector<Eigen::Vector3f> valid_voxels;
    for(size_t x_idx = 0; x_idx < x_node_num; x_idx++) {
        for(size_t y_idx = 0; y_idx < y_node_num; y_idx++) {
            for(size_t z_idx = 0; z_idx < z_node_num; z_idx++) {
                PointWithOccurrences& pt = voxels[x_idx][y_idx][z_idx];
                if(pt.n > 0) {
                    valid_voxels.push_back(pt.p / pt.n);
                }
            }
        }
    }

    _point_cloud.resize(valid_voxels.size(), 3);
    for(size_t i = 0; i < valid_voxels.size(); i++) {
        _point_cloud.row(i) = valid_voxels[i];
    }
    printf("Point size after voxel downsampled: %zu\n", _point_cloud.rows());
    setKDTree();
}


void PointCloudHandler::rmOutliersByRadius(float radius, int min_neighbor_num)
{
    PointCloudXYZ temp = _point_cloud;
    size_t count = 0;

    for(int i = 0; i < _point_cloud.rows(); i++){
        Eigen::Vector3f query_point = _point_cloud.row(i);

        std::vector<float> query_pt;
        for (int j = 0; j < 3; j++) {
            query_pt.push_back(query_point(j));
        }

        //search
        nanoflann::SearchParams params;
        std::vector<std::pair<Eigen::Index, float> > matches;
        _kd_tree->index->radiusSearch(&query_pt.at(0), radius, matches, params);

        // Filter outliers
        if((int)matches.size() < min_neighbor_num) continue;

        temp.row(count++) = _point_cloud.row(i);
    }
    _point_cloud = temp.topRows(count);
    printf("Point size after radius-filter: %zu\n", _point_cloud.rows());
    setKDTree();
}


void PointCloudHandler::rmOutliersByKNeighbors(int k, float max_neighbor_dis)
{
    PointCloudXYZ temp = _point_cloud;
    size_t count = 0;

    for(int i = 0; i < _point_cloud.rows(); i++){
        Eigen::Vector3f query_point = _point_cloud.row(i);

        std::vector<float> query_pt;
        for (int j = 0; j < 3; j++) {
            query_pt.push_back(query_point(j));
        }

        // set wtf vectors
        std::vector<size_t> ret_indexes(k);
        std::vector<float> out_dists_sqr(k);
        nanoflann::KNNResultSet<float> resultSet(k);
        resultSet.init(&ret_indexes.at(0), &out_dists_sqr.at(0));

        // knn search
        _kd_tree->index->findNeighbors(resultSet, &query_pt.at(0),
                                       nanoflann::SearchParams(k));

        // Filter outliers
        bool flag = true;
        for (int i = 0; i < k; i++) {
            Eigen::Vector3f pt = _point_cloud.row(ret_indexes.at(i));
            float dis = (pt - query_point).norm();
            if(dis >= max_neighbor_dis) {
                flag = false;
                break;
            }
        }
        if(flag) temp.row(count++) = _point_cloud.row(i);
    }
    _point_cloud = temp.topRows(count);
    printf("Point size after k-neighbors-filter: %zu\n", _point_cloud.rows());
    setKDTree();
}


const PointCloudXYZ& PointCloudHandler::getCurrentPointCloud() const
{
    return _point_cloud;
}


std::vector<size_t> PointCloudHandler::findKNeighbors(
        const Eigen::Vector3f &query_point, int k)
{
    std::vector<float> query_pt;
    for (int j = 0; j < 3; j++) {
        query_pt.push_back(query_point(j));
    }

    // set wtf vectors
    std::vector<size_t> indices(k);
    std::vector<float> out_dists_sqr(k);
    nanoflann::KNNResultSet<float> resultSet(k);
    resultSet.init(&indices.at(0), &out_dists_sqr.at(0));

    // knn search
    _kd_tree->index->findNeighbors(resultSet, &query_pt.at(0),
                                   nanoflann::SearchParams(k));

    return indices;
}


std::vector<size_t> PointCloudHandler::findNeighborsByRadius(
        const Eigen::Vector3f &query_point, float radius)
{
    std::vector<float> query_pt;
    for (int j = 0; j < 3; j++) {
        query_pt.push_back(query_point(j));
    }

    //search
    nanoflann::SearchParams params;
    std::vector<std::pair<Eigen::Index, float> > matches;
    _kd_tree->index->radiusSearch(&query_pt.at(0), radius, matches, params);

    // Filter indices
    std::vector<size_t> indices;
    for (size_t i = 0; i < matches.size(); i++)
    {
        indices.push_back(matches.at(i).first);
    }
    return indices;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudHandler::toPCLPointCloud() const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(
                new pcl::PointCloud<pcl::PointXYZ>);
    pcl_pc->resize(_point_cloud.rows());
    for(long i = 0; i < _point_cloud.rows(); i++) {
        Eigen::Vector3f pt = _point_cloud.row(i);
        pcl_pc->at(i) = {pt[0], pt[1], pt[2]};
    }

    return pcl_pc;
}


void PointCloudHandler::createMesh(pcl::PolygonMesh& mesh,
        float search_radius, float mu, int max_neighbors, bool use_mls)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points(
                this->toPCLPointCloud());
    printf("pcl point.size: %zu\n", pcl_points->size());

    pcl::PointCloud<pcl::PointNormal>::Ptr pcl_points_n(
                new pcl::PointCloud<pcl::PointNormal>);
    if (use_mls) {
        // MSL for smoothing
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (
                    new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(pcl_points);

        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        mls.setComputeNormals(true);
        mls.setInputCloud(pcl_points);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(search_radius);
        mls.process(*pcl_points_n);
        printf("Point size after mls: %zu\n", pcl_points_n->size());

        //util::pclPoints2EigenPoints(pcl_points_n, _point_cloud);
    }
    else{
        pcl_points_n = util::estimateNormal(pcl_points);
        printf("Point size after estimating normal: %zu\n", pcl_points_n->size());
    }

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
                new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(pcl_points_n);

    // ---- Create triangles ---
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setSearchRadius(search_radius);    // max edge length for every triangle
    gp3.setMu(mu); // maximum acceptable distance for a point to be considered as a neighbor
    gp3.setMaximumNearestNeighbors(max_neighbors);
    gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
    gp3.setMinimumAngle(M_PI/9.f); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud(pcl_points_n);
    gp3.setSearchMethod(tree2); printf("---\n");
    gp3.reconstruct(mesh);
}


void PointCloudHandler::setKDTree()
{
    if(_kd_tree) _kd_tree.reset();
    _kd_tree = KDTREE_PTR(3, _point_cloud);
}


/* -------------------------------------------------------------------------- */
/*                            namespace::util                                 */
/* -------------------------------------------------------------------------- */

namespace util {


void vectorPoints2EigenPoints(const std::vector<Eigen::Vector3f>& in,
                              PointCloudXYZ& out)
{
    out.resize(in.size(), 3);
    for(size_t i = 0; i < in.size(); i++) {
        out.row(i) = in[i];
    }
}


void eigenPoints2vectorPoints(const PointCloudXYZ& in,
                              std::vector<Eigen::Vector3f>& out)
{
    out.resize(in.rows());
    for(long i = 0; i < in.rows(); i++) {
        out[i] = in.row(i);
    }
}


void pclPoints2EigenPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr in,
                           PointCloudXYZ& out)
{
    out.resize(in->size(), 3);
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointNormal& pt = in->at(i);
        out.row(i) = Eigen::Vector3f(pt.x, pt.y, pt.z);
    }
}

} // namespace::util
