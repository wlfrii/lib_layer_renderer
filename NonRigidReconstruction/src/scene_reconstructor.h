#ifndef SCENE_RECONSTRUCTOR_H_LF
#define SCENE_RECONSTRUCTOR_H_LF
#include <opencv2/opencv.hpp>
#include <lib_math.h>
#include <lib_layer_renderer.h>
#include <vector>

class SGM;

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
                       bool is_seqc = false, uint8_t pyd_times = 2);
    ~SceneReconstructor();


    /**
     * @brief Update 3D texture of the input stereo image
     * @param l_image
     * @param r_image
     */
    void reconstruct(const cv::Mat &l_image, const cv::Mat &r_image);


    void plot();

private:
    /**
     * @brief Calculate the RGBD for each valid pixel
     */
    void calcRGBD(const cv::Mat &l_image, const cv::Mat &r_image);


    /**
     * @brief Filter out some points based on point density threshold
     */
    void filterVertices();


    std::vector<std::pair<cv::Point2i, cv::Point2i>>
    keyPointMatching(const cv::Mat &left_tex);
    void cvProcess();

private:
    mmath::CameraProjector _cam_proj;

    bool _is_seqc;        //!< Wheter create texture for a sequence

    SGM* _sgm;            //!< SGM object
    float *_disparity;    //!< Calculated disparity
    uint8_t _pyd_times;   //!< Times of downsampling
    uint8_t _step;        //!< Step determined based on _pyd_times.
    uint16_t _width;      //!< Width of the pyd texture
    uint16_t _height;     //!< height of the pyd texture

    cv::Mat _texture;     //!< The default texture is set to left_tex

    cv::Mat _rgbd;        //!< [R, G, B, disparoty]

    std::vector<mlayer::Vertex3D> _vertices_3d;

    mlayer::LayerRenderer*  _layer_renderer;
    std::shared_ptr<mlayer::LayerTexture3D> _layer_texture3d;
};

#endif // SCENE_RECONSTRUCTOR_H_LF
