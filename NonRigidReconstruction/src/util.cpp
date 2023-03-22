#include "util.h"
#include <nanoflann.hpp>
#include <pcl/registration/icp.h>

#define VERBOSE_DOWNSAMPLING 0

namespace util {

struct PointWithOccurrences {
    PointWithOccurrences() : p({0, 0, 0}), n(0) {}
    Eigen::Vector3f p;  // The position of the point
    size_t          n;  // The number of occurrences
};
void voxelDownSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr in, float voxel_size,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    Eigen::Vector3f max_xyzf(0, 0, 0);
    Eigen::Vector3f min_xyzf(1000, 1000, 1000);
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointXYZ& pt = in->at(i);
        if(max_xyzf[0] < pt.x) max_xyzf[0] = pt.x;
        if(max_xyzf[1] < pt.y) max_xyzf[1] = pt.y;
        if(max_xyzf[2] < pt.z) max_xyzf[2] = pt.z;
        if(min_xyzf[0] > pt.x) min_xyzf[0] = pt.x;
        if(min_xyzf[1] > pt.y) min_xyzf[1] = pt.y;
        if(min_xyzf[2] > pt.z) min_xyzf[2] = pt.z;
    }
    size_t x_node_num = floor((max_xyzf[0] - min_xyzf[0]) / voxel_size) + 1;
    size_t y_node_num = floor((max_xyzf[1] - min_xyzf[1]) / voxel_size) + 1;
    size_t z_node_num = floor((max_xyzf[2] - min_xyzf[2]) / voxel_size) + 1;

#if VERBOSE_DOWNSAMPLING
    printf("x_range:[%f, %f]\n", min_xyzf[0], max_xyzf[0]);
    printf("y_range:[%f, %f]\n", min_xyzf[1], max_xyzf[1]);
    printf("z_range:[%f, %f]\n", min_xyzf[2], max_xyzf[2]);
    printf("voxel size: %zux%zux%zu\n", x_node_num, y_node_num, z_node_num);
#endif

    // Group vertices into voxels
    std::vector<std::vector<std::vector<PointWithOccurrences>>> voxels(
                x_node_num, std::vector<std::vector<PointWithOccurrences>>(
                    y_node_num, std::vector<PointWithOccurrences>(
                        z_node_num, PointWithOccurrences())));
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointXYZ& pt = in->at(i);
        int x_idx = floor((pt.x - min_xyzf[0]) / voxel_size);
        int y_idx = floor((pt.y - min_xyzf[1]) / voxel_size);
        int z_idx = floor((pt.z - min_xyzf[2]) / voxel_size);

//        printf("index: %dx%dx%d\n", x_idx, y_idx, z_idx);

        voxels[x_idx][y_idx][z_idx].p += Eigen::Vector3f(pt.x, pt.y, pt.z);
        voxels[x_idx][y_idx][z_idx].n++;
    }

    // Filter out the valid voxels
    out->clear();
    for(size_t x_idx = 0; x_idx < x_node_num; x_idx++) {
        for(size_t y_idx = 0; y_idx < y_node_num; y_idx++) {
            for(size_t z_idx = 0; z_idx < z_node_num; z_idx++) {
                PointWithOccurrences& pt = voxels[x_idx][y_idx][z_idx];
                if(pt.n > 0) {
                    pt.p /= pt.n;
                    out->push_back({pt.p[0], pt.p[1], pt.p[2]});
                }
            }
        }
    }
}


void voxelDownSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, float voxel_size,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    Eigen::Vector3f max_xyzf(0, 0, 0);
    Eigen::Vector3f min_xyzf(1000, 1000, 1000);
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointXYZRGB& pt = in->at(i);
        if(max_xyzf[0] < pt.x) max_xyzf[0] = pt.x;
        if(max_xyzf[1] < pt.y) max_xyzf[1] = pt.y;
        if(max_xyzf[2] < pt.z) max_xyzf[2] = pt.z;
        if(min_xyzf[0] > pt.x) min_xyzf[0] = pt.x;
        if(min_xyzf[1] > pt.y) min_xyzf[1] = pt.y;
        if(min_xyzf[2] > pt.z) min_xyzf[2] = pt.z;
    }
    size_t x_node_num = floor((max_xyzf[0] - min_xyzf[0]) / voxel_size) + 1;
    size_t y_node_num = floor((max_xyzf[1] - min_xyzf[1]) / voxel_size) + 1;
    size_t z_node_num = floor((max_xyzf[2] - min_xyzf[2]) / voxel_size) + 1;
#if VERBOSE_DOWNSAMPLING
    printf("x_range:[%f, %f]\n", min_xyzf[0], max_xyzf[0]);
    printf("y_range:[%f, %f]\n", min_xyzf[1], max_xyzf[1]);
    printf("z_range:[%f, %f]\n", min_xyzf[2], max_xyzf[2]);
    printf("voxel size: %zux%zux%zu\n", x_node_num, y_node_num, z_node_num);
#endif

    // Group vertices into voxels
    std::vector<std::vector<std::vector<PointWithOccurrences>>> voxels(
                x_node_num, std::vector<std::vector<PointWithOccurrences>>(
                    y_node_num, std::vector<PointWithOccurrences>(
                        z_node_num, PointWithOccurrences())));
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointXYZRGB& pt = in->at(i);
        int x_idx = floor((pt.x - min_xyzf[0]) / voxel_size);
        int y_idx = floor((pt.y - min_xyzf[1]) / voxel_size);
        int z_idx = floor((pt.z - min_xyzf[2]) / voxel_size);

        //printf("index: %dx%dx%d\n", x_idx, y_idx, z_idx);

        voxels[x_idx][y_idx][z_idx].p += Eigen::Vector3f(pt.x, pt.y, pt.z);
        voxels[x_idx][y_idx][z_idx].n++;
    }

    // Filter out the valid voxels
    out->clear();
    for(size_t x_idx = 0; x_idx < x_node_num; x_idx++) {
        for(size_t y_idx = 0; y_idx < y_node_num; y_idx++) {
            for(size_t z_idx = 0; z_idx < z_node_num; z_idx++) {
                PointWithOccurrences& pt = voxels[x_idx][y_idx][z_idx];
                if(pt.n > 0) {
                    pt.p /= pt.n;
                    out->push_back({pt.p[0], pt.p[1], pt.p[2]});
                }
            }
        }
    }
}


void voxelDownSampling(pcl::PointCloud<pcl::PointNormal>::Ptr in, float voxel_size,
                       pcl::PointCloud<pcl::PointNormal>::Ptr out)
{
    Eigen::Vector3f max_xyzf(0, 0, 0);
    Eigen::Vector3f min_xyzf(1000, 1000, 1000);
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointNormal& pt = in->at(i);
        if(max_xyzf[0] < pt.x) max_xyzf[0] = pt.x;
        if(max_xyzf[1] < pt.y) max_xyzf[1] = pt.y;
        if(max_xyzf[2] < pt.z) max_xyzf[2] = pt.z;
        if(min_xyzf[0] > pt.x) min_xyzf[0] = pt.x;
        if(min_xyzf[1] > pt.y) min_xyzf[1] = pt.y;
        if(min_xyzf[2] > pt.z) min_xyzf[2] = pt.z;
    }
    size_t x_node_num = floor((max_xyzf[0] - min_xyzf[0]) / voxel_size) + 1;
    size_t y_node_num = floor((max_xyzf[1] - min_xyzf[1]) / voxel_size) + 1;
    size_t z_node_num = floor((max_xyzf[2] - min_xyzf[2]) / voxel_size) + 1;
#if VERBOSE_DOWNSAMPLING
    printf("x_range:[%f, %f]\n", min_xyzf[0], max_xyzf[0]);
    printf("y_range:[%f, %f]\n", min_xyzf[1], max_xyzf[1]);
    printf("z_range:[%f, %f]\n", min_xyzf[2], max_xyzf[2]);
    printf("voxel size: %zux%zux%zu\n", x_node_num, y_node_num, z_node_num);
#endif

    // Group vertices into voxels
    std::vector<std::vector<std::vector<PointWithOccurrences>>> voxels(
                x_node_num, std::vector<std::vector<PointWithOccurrences>>(
                    y_node_num, std::vector<PointWithOccurrences>(
                        z_node_num, PointWithOccurrences())));
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointNormal& pt = in->at(i);
        int x_idx = floor((pt.x - min_xyzf[0]) / voxel_size);
        int y_idx = floor((pt.y - min_xyzf[1]) / voxel_size);
        int z_idx = floor((pt.z - min_xyzf[2]) / voxel_size);

        //printf("index: %dx%dx%d\n", x_idx, y_idx, z_idx);

        voxels[x_idx][y_idx][z_idx].p += Eigen::Vector3f(pt.x, pt.y, pt.z);
        voxels[x_idx][y_idx][z_idx].n++;
    }

    // Filter out the valid voxels
    out->clear();
    for(size_t x_idx = 0; x_idx < x_node_num; x_idx++) {
        for(size_t y_idx = 0; y_idx < y_node_num; y_idx++) {
            for(size_t z_idx = 0; z_idx < z_node_num; z_idx++) {
                PointWithOccurrences& pt = voxels[x_idx][y_idx][z_idx];
                if(pt.n > 0) {
                    pt.p /= pt.n;
                    out->push_back({pt.p[0], pt.p[1], pt.p[2]});
                }
            }
        }
    }
}


struct VertexWithOccurrences {
    VertexWithOccurrences() : coord({0, 0, 0}), n(0) {}
    Eigen::Vector3f coord;  // The position of the point
    Eigen::Vector3f normal;  // The position of the point
    Eigen::Vector3f color;  // The position of the point
    size_t          n;  // The number of occurrences
};
void voxelDownSampling(const Vertices& in, float voxel_size, Vertices& out)
{
    Eigen::Vector3f max_xyzf(0, 0, 0);
    Eigen::Vector3f min_xyzf(1000, 1000, 1000);
    for(size_t i = 0; i < in.size(); i++) {
        const pcl::PointXYZ& pt = in.coords->at(i);
        if(max_xyzf[0] < pt.x) max_xyzf[0] = pt.x;
        if(max_xyzf[1] < pt.y) max_xyzf[1] = pt.y;
        if(max_xyzf[2] < pt.z) max_xyzf[2] = pt.z;
        if(min_xyzf[0] > pt.x) min_xyzf[0] = pt.x;
        if(min_xyzf[1] > pt.y) min_xyzf[1] = pt.y;
        if(min_xyzf[2] > pt.z) min_xyzf[2] = pt.z;
    }
    size_t x_node_num = floor((max_xyzf[0] - min_xyzf[0]) / voxel_size) + 1;
    size_t y_node_num = floor((max_xyzf[1] - min_xyzf[1]) / voxel_size) + 1;
    size_t z_node_num = floor((max_xyzf[2] - min_xyzf[2]) / voxel_size) + 1;
#if VERBOSE_DOWNSAMPLING
    printf("x_range:[%f, %f]\n", min_xyzf[0], max_xyzf[0]);
    printf("y_range:[%f, %f]\n", min_xyzf[1], max_xyzf[1]);
    printf("z_range:[%f, %f]\n", min_xyzf[2], max_xyzf[2]);
    printf("voxel size: %zux%zux%zu\n", x_node_num, y_node_num, z_node_num);
#endif

    // Group vertices into voxels
    std::vector<std::vector<std::vector<VertexWithOccurrences>>> voxels(
                x_node_num, std::vector<std::vector<VertexWithOccurrences>>(
                    y_node_num, std::vector<VertexWithOccurrences>(
                        z_node_num, VertexWithOccurrences())));
    for(size_t i = 0; i < in.size(); i++) {
        const Vertex& vert = in[i];
        const pcl::PointXYZ& pt = vert.pclCoord();
        int x_idx = floor((pt.x - min_xyzf[0]) / voxel_size);
        int y_idx = floor((pt.y - min_xyzf[1]) / voxel_size);
        int z_idx = floor((pt.z - min_xyzf[2]) / voxel_size);

        //printf("index: %dx%dx%d\n", x_idx, y_idx, z_idx);

        voxels[x_idx][y_idx][z_idx].coord += vert.coord;
        voxels[x_idx][y_idx][z_idx].normal += vert.normal;
        voxels[x_idx][y_idx][z_idx].color += vert.color;
        voxels[x_idx][y_idx][z_idx].n++;
    }

    // Filter out the valid voxels
    out.clear();
    for(size_t x_idx = 0; x_idx < x_node_num; x_idx++) {
        for(size_t y_idx = 0; y_idx < y_node_num; y_idx++) {
            for(size_t z_idx = 0; z_idx < z_node_num; z_idx++) {
                VertexWithOccurrences& pt = voxels[x_idx][y_idx][z_idx];
                if(pt.n > 0) {
                    pt.coord /= pt.n;
                    pt.normal /= pt.n;
                    pt.color /= pt.n;
                    out.push_back({pt.coord, pt.normal, pt.color});
                }
            }
        }
    }
}


pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr estimateNormal(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
                new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(pt_cloud);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(pt_cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(
                new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    //printf("normal.size:%zu\n", normals->size());

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(
                new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //* cloud_with_normals = cloud + normals
    pcl::concatenateFields(*pt_cloud, *normals, *cloud_with_normals);
    //printf("cloud_with_normals.size:%zu\n", cloud_with_normals->size());

    return cloud_with_normals;
}


pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormal(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
                new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pt_cloud);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pt_cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(
                new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    //printf("normal.size:%zu\n", normals->size());

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
                new pcl::PointCloud<pcl::PointNormal>);
    //* cloud_with_normals = cloud + normals
    pcl::concatenateFields(*pt_cloud, *normals, *cloud_with_normals);
    //printf("cloud_with_normals.size:%zu\n", cloud_with_normals->size());

    return cloud_with_normals;
}


void estimateImageNormal(const cv::Mat& gray_image, cv::Mat& normal_image)
{
    normal_image = cv::Mat(gray_image.size(), CV_32FC3);

    for(int v = 1; v < gray_image.rows - 1; ++v) {
        for(int u = 1; u < gray_image.cols - 1; ++u) {

            float dzdx = (gray_image.at<float>(v + 1, u) -
                          gray_image.at<float>(v - 1, u)) / 2.0;
            float dzdy = (gray_image.at<float>(v, u + 1) -
                          gray_image.at<float>(v, u - 1)) / 2.0;

            cv::Vec3f d(dzdx, dzdy, 1.0f);
            cv::Vec3f n = cv::normalize(d);

            normal_image.at<cv::Vec3f>(v, u) = n;
        }
    }
}

cv::Mat im2uchar(const cv::Mat &image)
{
    cv::Mat out;
    if(image.type() == CV_32FC1 || image.type() == CV_64FC1)
        out = cv::Mat(image.size(), CV_8UC1);
    else if(image.type() == CV_32FC3 || image.type() == CV_64FC3)
        out = cv::Mat(image.size(), CV_8UC3);

    if(image.depth() == CV_32F)
    {
        for(int i = 0; i < image.rows; i++){
            const float* row_data_in = image.ptr<float>(i);
            uchar* row_data_out = out.ptr<uchar>(i);
            for(int j = 0; j < image.cols * image.channels(); j++){
                row_data_out[j] = (uchar)MIN(row_data_in[j] * 255.0, 255);
            }
        }
    }
    else if(image.depth() == CV_64F)
    {
        for(int i = 0; i < image.rows; i++){
            const double* row_data_in = image.ptr<double>(i);
            uchar* row_data_out = out.ptr<uchar>(i);
            for(int j = 0; j < image.cols * image.channels(); j++){
                row_data_out[j] = (uchar)MIN(row_data_in[j] * 255.0, 255);
            }
        }
    }

    return out;
}


void estimateTransform(
        const std::vector<
        std::pair<pcl::PointXYZ, pcl::PointXYZ>>& correspondences,
        Eigen::Matrix4f& transform)
{
    transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    estimateTransform(correspondences, R, t);
    transform.topLeftCorner(3, 3) = R;
    transform.topRightCorner(3, 1) = t;
}


void estimateTransform(
        const std::vector<
        std::pair<pcl::PointXYZ, pcl::PointXYZ>>& correspondences,
        Eigen::Matrix3f& R, Eigen::Vector3f& t)
{
    size_t N = correspondences.size();
# if 0
    // ICP is time consuming
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev(
                new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_curr(
                new pcl::PointCloud<pcl::PointXYZ>);
    cloud_prev->resize(N);
    cloud_curr->resize(N);

    for(size_t i = 0; i < correspondences.size(); i++) {
        cloud_prev->at(i) = correspondences[i].first;
        cloud_curr->at(i) = correspondences[i].second;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(100);
    icp.setInputSource(cloud_prev);
    icp.setInputTarget(cloud_curr);
    icp.align(*cloud_prev);
    if(icp.hasConverged()){
        transform = icp.getFinalTransformation().cast<float>();
        std::cout << transform << "\n";
    }
    else{
        printf("ICP error\n");
        std::exit(EXIT_FAILURE);
    }
#else
    Eigen::MatrixXf prev, curr;
    prev.resize(correspondences.size(), 3);
    curr.resize(correspondences.size(), 3);
    for(size_t i = 0; i < correspondences.size(); i++) {
        prev.row(i) = correspondences[i].first.getVector3fMap();
        curr.row(i) = correspondences[i].second.getVector3fMap();
    }
    Eigen::RowVector3f centroid_prev = {
        prev.col(0).mean(), prev.col(1).mean(), prev.col(2).mean()};
    Eigen::RowVector3f centroid_curr = {
        curr.col(0).mean(), curr.col(1).mean(), curr.col(2).mean()};

    Eigen::MatrixXf H = (curr - centroid_curr.replicate(N, 1)).transpose() *
            (prev - centroid_prev.replicate(N, 1));
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
                H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf V = svd.matrixV(), U = svd.matrixU();

    R = V*U.transpose();
    if(R.determinant() < 0) {
        Eigen::MatrixXf tmp = V;
        V.col(2) = -tmp.col(2);
        R = V*U.transpose();
    }

    t = -R * centroid_prev.transpose() + centroid_curr.transpose();
#endif
}

} // namespace::util


