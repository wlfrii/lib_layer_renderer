/**
 * To facilate the using of PCL functions.
 */
#ifndef POINT_CLOUD_DEFINE_H_LF
#define POINT_CLOUD_DEFINE_H_LF
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/common/io.h> // for concatenateFields

const float VERTEX_MAX_WEIGHT = 5.f;
const float VERTEX_DENSITY = 0.25f;

const float TAO_WEIGHT = 3;
const int TAO_TIME = 10;


struct Vertex
{
    Vertex() : coord(Eigen::Vector3f::Zero()), normal(Eigen::Vector3f::Zero())
      , color(Eigen::Vector3f::Zero()), weight(0), timestamp(0), stability(0)
    {}

    Eigen::Vector3f coord;
    Eigen::Vector3f normal;
    Eigen::Vector3f color;
    float weight;
    size_t timestamp;
    bool stability;

    const pcl::PointXYZ& pclCoord() const {
        static pcl::PointXYZ pt;
        pt.getVector3fMap() = coord;
        return pt;
    }
};


class Vertices : public std::vector<Vertex>
{
public:
    Vertices() {}

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud() const {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(
                    new pcl::PointCloud<pcl::PointXYZ>);
        pcl_pc->resize(this->size());
        for(size_t i = 0; i < this->size(); i++) {
            pcl_pc->at(i).getVector3fMap() = this->at(i).coord;
        }
        return pcl_pc;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr pclPointCloudWithNormals() const {
        pcl::PointCloud<pcl::PointNormal>::Ptr pcl_pc(
                    new pcl::PointCloud<pcl::PointNormal>);
        pcl_pc->resize(this->size());
        for(size_t i = 0; i < this->size(); i++) {
            pcl_pc->at(i).getVector3fMap() = this->at(i).coord;
            pcl_pc->at(i).getNormalVector3fMap() = this->at(i).normal;
        }
        return pcl_pc;
    }

};


//struct Vertices{
//    Vertices()
//        : coords(new pcl::PointCloud<pcl::PointXYZ>)
//    {}

//    const Vertex& operator[] (size_t index) const {
//        if(index > this->size()) {
//            printf("Abort: Index out of vector range.\n");
//            std::abort();
//        }
//        static Vertex vert;
//        vert.coord = coords->at(index).getVector3fMap();
//        vert.normal = normals[index];
//        vert.color = colors[index];
//        vert.weight = weights[index];
//        vert.timestamp = timestamps[index];
//        vert.stability = stabilitys[index];

//        return vert;
//    }

//    size_t size() const {
//        sizeCheck();
//        return coords->size();
//    }

//    void resize(size_t n) {
//        coords->resize(n);
//        normals.resize(n);
//        colors.resize(n);
//        weights.resize(n);
//        timestamps.resize(n);
//        stabilitys.resize(n);
//    }

//    void clear() {
//        coords->clear();
//        normals.clear();
//        colors.clear();
//        weights.clear();
//        timestamps.clear();
//        stabilitys.clear();
//    }

//    void push_back(const Vertex& vert) {
//        coords->push_back(vert.pclCoord());
//        normals.push_back(vert.normal);
//        colors.push_back(vert.color);
//        weights.push_back(vert.weight);
//        timestamps.push_back(vert.timestamp);
//        stabilitys.push_back(vert.stability);
//    }

//    pcl::PointCloud<pcl::PointNormal>::Ptr coordsWithNormals() const {
//        pcl::PointCloud<pcl::PointNormal>::Ptr ret(
//                    new pcl::PointCloud<pcl::PointNormal>);

//        pcl::PointCloud<pcl::Normal>::Ptr pclnormals(
//                    new pcl::PointCloud<pcl::Normal>);
//        pclnormals->resize(normals.size());
//        for(size_t i = 0; i < normals.size(); i++) {
//            pclnormals->at(i).getNormalVector3fMap() = normals[i];
//        }

//        pcl::concatenateFields(*coords, *pclnormals, *ret);

//        return ret;
//    }



//    pcl::PointCloud<pcl::PointXYZ>::Ptr coords;
//    std::vector<Eigen::Vector3f> normals;
//    std::vector<Eigen::Vector3f> colors;
//    std::vector<float> weights;
//    std::vector<size_t> timestamps;
//    std::vector<bool> stabilitys;

//private:
//    void sizeCheck() const {
//        if(coords->size() != normals.size() || normals.size() != colors.size()){
//            printf("Abort: Different data size!. Assert at "
//                   "'coords->size() != normals.size() || "
//                   "normals.size() != colors.size()'\n"
//                   "Actually coords->size()=%zu, normals.size()=%zu, "
//                   "colors.size()=%zu\n", coords->size(),
//                   normals.size(), colors.size());
//            std::abort();
//        }
//    }
//};


#endif // POINT_CLOUD_DEFINE_H_LF
