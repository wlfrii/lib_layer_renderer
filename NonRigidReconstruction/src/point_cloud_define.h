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
        , coords_with_normals(new pcl::PointCloud<pcl::PointNormal>)
    {}

    pcl::PointCloud<pcl::PointXYZ>::Ptr coords;
    pcl::PointCloud<pcl::PointNormal>::Ptr coords_with_normals;
    std::vector<Eigen::Vector3f> colors;

    const Vertex& operator[] (size_t index) const {
        static Vertex vert;
        if(coords_with_normals->size() != colors.size()){
            printf("Abort: Different data size!. Assert at "
                   "'coords_with_normals->size() != normals.size()'\n");
            std::abort();
        }
        if(index > coords_with_normals->size()) {
            printf("Abort: Index out of vector range.\n");
            std::abort();
        }
        vert.coord = coords->at(index);
        vert.normal = coords_with_normals->at(index).getNormalVector3fMap();
        vert.color = colors[index];

        return vert;
    }
};


#endif // POINT_CLOUD_DEFINE_H_LF
