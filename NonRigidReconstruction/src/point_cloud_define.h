/**
 * To facilate the using of PCL functions.
 */
#ifndef POINT_CLOUD_DEFINE_H_LF
#define POINT_CLOUD_DEFINE_H_LF
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>


struct Vertex
{
    pcl::PointXYZ coord;
    Eigen::Vector3f normal;
    Eigen::Vector3f color;
};



struct Vertices{
    Vertices()
        : coords(new pcl::PointCloud<pcl::PointXYZ>)
    {}

    pcl::PointCloud<pcl::PointXYZ>::Ptr coords;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector3f> colors;

    const Vertex& operator[] (size_t index) const {
        static Vertex vert;
        if(coords->size() != normals.size() || normals.size() != colors.size()){
            printf("Abort: Different data size!. Assert at 'coords->size() != "
                   "normals.size() || normals.size() != colors.size()'\n");
            std::abort();
        }
        if(index > coords->size()) {
            printf("Abort: Index out of vector range.\n");
            std::abort();
        }
        vert.coord = coords->at(index);
        vert.normal = normals[index];
        vert.color = colors[index];

        return vert;
    }
};


#endif // POINT_CLOUD_DEFINE_H_LF
