#ifndef PCL_VIEWER_H_LF
#define PCL_VIEWER_H_LF
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


class PCLViewer
{
protected:
    PCLViewer()
        : _viewer(pcl::visualization::PCLVisualizer("PCL Viewer"))
    {
        _viewer.setBackgroundColor(0.2, 0.3, 0.3, 0);
    }
public:
    static PCLViewer* instance() {
        static PCLViewer viewer;
        return &viewer;
    }

    void view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud,
              const std::string& id = "pt_cloud", uint8_t point_size = 3) {
        _viewer.addPointCloud(pt_cloud, id);
        _viewer.setPointCloudRenderingProperties (
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size);
        while(!_viewer.wasStopped()) {
            _viewer.spinOnce();
        }
        _viewer.removePointCloud(id);
    }

    void view(pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud,
              const std::string& id = "pt_cloud", uint8_t point_size = 3) {
        _viewer.addPointCloud(pt_cloud, id);
        _viewer.setPointCloudRenderingProperties (
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size);
        while(!_viewer.wasStopped()) {
            _viewer.spinOnce();
        }
        _viewer.removePointCloud(id);
    }

    void view(const pcl::PolygonMesh& mesh,
              const std::string& id = "mesh") {
        _viewer.addPolygonMesh(mesh, id);
        while(!_viewer.wasStopped()) {
            _viewer.spinOnce();
        }
        _viewer.removePolygonMesh(id);
    }

private:
    pcl::visualization::PCLVisualizer _viewer;
};

#endif // PCL_VIEWER_H_LF
