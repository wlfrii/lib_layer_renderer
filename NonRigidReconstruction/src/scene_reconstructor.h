#ifndef SCENE_RECONSTRUCTOR_H_LF
#define SCENE_RECONSTRUCTOR_H_LF
#include <opencv2/opencv.hpp>
#include <lib_math.h>
#include <lib_layer_renderer.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "point_cloud_handler.h"


class SGM;
class EmbeddedDeformation;

class SceneReconstructor
{
public:
    /**
     * @brief Constructor of class LayerTexture3D.
     * @param fxy        Focal length of the camera.
     * @param cx         X coordinates of camera optical center.
     * @param cy         Y coordinates of camera optical center.
     * @param t          Distance between binocular camera.
     * @param is_seqc    Initlize this object for a sequence.
     * @param pyd_times  Times of downsampling for the texture.
     */
    SceneReconstructor(const mmath::CameraProjector& cam_proj,
                       bool is_seqc = false);
    ~SceneReconstructor();


    /**
     * @brief Update 3D texture of the input stereo image
     * @param l_image
     * @param r_image
     */
    void reconstruct(const cv::Mat &l_image, const cv::Mat &r_image);


    void plot();

    cv::Mat getPlotResult();

private:
    void calcDepthMap(const cv::Mat &l_image, const cv::Mat &r_image);

    void filterPointCloud();

    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>
    keyPointMatching(const cv::Mat &prev_image, const cv::Mat &curr_image);


    /**
     * @brief Filter out some points based on point density threshold
     */
//    void filterVertices();

private:
    const std::shared_ptr<mmath::CameraProjector> _cam_proj;
    const mmath::cam::ID _cam_id;

    bool _is_seqc;        //!< Wheter create texture for a sequence

    SGM* _sgm;            //!< SGM object
    float *_disparity;    //!< Calculated disparity
    uint8_t _pyd_times;   //!< Times of downsampling
    uint8_t _step;        //!< Step determined based on _pyd_times.
    uint16_t _width;      //!< Width of the pyd texture
    uint16_t _height;     //!< height of the pyd texture

    cv::Mat _texture;     //!< The default texture is set to left_tex
    cv::Mat _depthmap;    //!< [R, G, B, disparoty]
    uint16_t _u_start;
    uint16_t _u_end;

    cv::Mat _prev_depthmap;
    cv::Mat _prev_image;

    std::vector<mlayer::Vertex3D> _vertices_3d;
    std::vector<mlayer::Vertex3D> _traingles_3d;

    std::shared_ptr<mlayer::LayerRenderer>  _layer_renderer;
    std::array<mlayer::LayerTexture3D::Ptr, 4> _layer_texture3d;
    mlayer::LayerBackground::Ptr _layer_l_image;
    mlayer::LayerBackground::Ptr _layer_r_image;

    std::shared_ptr<PointCloudHandler> _pc_handler;
    std::shared_ptr<EmbeddedDeformation> _ed;
};

#endif // SCENE_RECONSTRUCTOR_H_LF
