#ifndef LAYER_TEXTURE3D_H_LF
#define LAYER_TEXTURE3D_H_LF
#include "layer_model.h"
#include <opencv2/opencv.hpp>

class SGM;

class LayerTexture3D : public LayerModel
{
public:
    LayerTexture3D(const cv::Mat &left_tex, const cv::Mat &right_tex,
                   float fxy, float cx, float cy, float t,
                   uint8_t pyd_times = 2);
    ~LayerTexture3D();

    void update3DTexture(const cv::Mat &left_tex, const cv::Mat &right_tex);

private:
    float _fxy;
    float _cx;
    float _cy;
    float _t;

    SGM* _sgm;
    float *_disparity;

    uint8_t _pyd_times;
    uint16_t _width;
    uint16_t _height;

    cv::Mat _texture;
    cv::Mat _left_gray;
    cv::Mat _right_gray;
};

#endif // LAYER_TEXTURE3D_H_LF
