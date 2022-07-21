#include "../include/layer_texture3d.h"
#include "sgm/sgm.h"
#include <lib_math/lib_math.h>
#include <gl_util.h>
#include <global.h>
#include "layer_define.h"

namespace {
bool isNear(float d1, float d2, float d3, float thresh = 2.f)
{
    float dmax = std::fmaxf(d1, std::fmaxf(d2, d3));
    float dmin = std::fminf(d1, std::fminf(d2, d3));
    if(dmax - dmin < thresh){
        return true;
    }
    else return false;
};

cv::Mat im3d;
}

LayerTexture3D::LayerTexture3D(const cv::Mat &left_tex, const cv::Mat &right_tex,
                               float fxy, float cx, float cy, float t,
                               uint8_t pyd_times)
    : LayerModel(LAYER_TEXTURE3D, glm::vec3(0.f, 0.f, 0.f))
    , _fxy(fxy), _cx(cx), _cy(cy), _t(t)
    , _sgm(new SGM())
    , _pyd_times(pyd_times)
{
    SGM::Option sgm_option;
    sgm_option.aggr_path_num = 4;
    sgm_option.min_disparity = 1;
    sgm_option.max_disparity = 64;
    sgm_option.census_block_radius = 5;
    sgm_option.is_check_lr = true;
    sgm_option.lr_check_thresh = 1.0f;
    sgm_option.is_check_uniqueness = true;
    sgm_option.uniqueness_ratio = 0.99;
    sgm_option.is_remove_speckles = true;
    sgm_option.min_speckle_area = 50;
    sgm_option.P1 = 10;
    sgm_option.P2 = 150;
    sgm_option.is_fill_holes = false;

    assert(left_tex.rows == right_tex.rows && left_tex.cols == right_tex.cols);

    _width = left_tex.cols;
    _height = left_tex.rows;
    im3d = cv::Mat(_height, _width, CV_32FC4); // [depth, R, G, B]

    for(int i = 0; i < pyd_times; i++){
        _width /= 2;
        _height /= 2;
    }

    bool flag = _sgm->initialize(_width, _height, sgm_option);
    if(!flag) EV_LOG("LayerTexture3D initialize SGM: %d\n", flag);

    _disparity = (float*)malloc(_width * _height * sizeof(float));

    update3DTexture(left_tex, right_tex);
}


LayerTexture3D::~LayerTexture3D()
{
    if(_sgm){
        delete _sgm;
        _sgm = nullptr;
    }
    free(_disparity);
}


void LayerTexture3D::update3DTexture(const cv::Mat &left_tex,
                                     const cv::Mat &right_tex)
{
    _texture = left_tex;
    cv::cvtColor(left_tex, _left_gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(right_tex, _right_gray, cv::COLOR_RGB2GRAY);

    for(int i = 0; i < _pyd_times; i++){
        cv::pyrDown(_left_gray, _left_gray);
        cv::pyrDown(_right_gray, _right_gray);
    }
    cv::boxFilter(_left_gray, _left_gray, -1, cv::Size(9,9));
    cv::boxFilter(_right_gray, _right_gray, -1, cv::Size(9,9));

    if(!_sgm->match(_left_gray.data, _right_gray.data, _disparity)){
        return;
    }

    static uint16_t count = 0;
    bool is_print = false;
    if(++count > 320) is_print = true;

    int k_pyrup = pow(2, _pyd_times);
    cv::Mat disp_mat = cv::Mat(_height, _width, CV_32FC1, cv::Scalar(0.f));
    for (int32_t i = 0; i < _height; i++) {
        for (int32_t j = 0; j < _width; j++) {
            const float disp = k_pyrup * _disparity[i * _width + j];
            if (disp != SGM::FLOAT_INF) {
                disp_mat.at<float>(i, j) = disp;
            }
        }
    }
    double min_disp = _width, max_disp = 0;
    cv::minMaxIdx(disp_mat, &min_disp, &max_disp);
    if(is_print) printf("Disparity range in [%f, %f].\t", min_disp, max_disp);

    std::vector<int> xs = mmath::linspace<int>(floor(min_disp), 1, floor(max_disp));
    std::vector<int> ys(floor(max_disp) - floor(min_disp) + 1);
    for(auto& y : ys) y = 0;
    for (int32_t i = 0; i < _height; i++) {
        for (int32_t j = 0; j < _width; j++) {
            float disp = disp_mat.at<float>(i, j);
            if (disp > min_disp) {
                ys[floor(disp) - floor(min_disp)]++;
            }
        }
    }
    if(is_print) printf("xs.size: %zu, ys.size:%zu.\t", xs.size(), ys.size());
    mmath::GaussianCurvef gauss = mmath::fitGuassianCurve<float>(xs, ys);
    if(is_print) printf("Gauss: mu:%f, sigma:%f, a:%f\n", gauss.mu, gauss.sigma, gauss.a);

    for(int i = 0; i < _pyd_times; i++){
        cv::pyrUp(disp_mat, disp_mat);
    }

    std::vector<VertexC> vertices;
    auto addVertex = [this, &vertices](int u, int v,
            const cv::Vec4f &drgb) {
        float depth = this->_t * this->_fxy / drgb[0];
        float x = (u - this->_cx) / this->_fxy * depth;
        float y = (v - this->_cy) / this->_fxy * depth;

        glm::vec4 position(x, y, depth, 1.f);
        glm::vec4 color(drgb[1], drgb[2], drgb[3], 1.f);

        vertices.push_back({position, color});
    };
    int step = 2;
    for (int32_t v = 0; v < disp_mat.rows; v += step) {
        for (int32_t u = 0; u < disp_mat.cols; u += step) {
            float disp = disp_mat.at<float>(v, u);

            bool condition = abs(disp - gauss.mu) <= 2*gauss.sigma;

            if (condition) {
                cv::Vec4f drgb = im3d.at<cv::Vec4f>(v, u);
                const cv::Vec3b pixel = _texture.at<cv::Vec3b>(v, u);
                float r = 1.f*pixel[0]/255.f;
                float g = 1.f*pixel[1]/255.f;
                float b = 1.f*pixel[2]/255.f;
                if(drgb[0] < 1){
                    im3d.at<cv::Vec4f>(v, u) = cv::Vec4f(disp, r, g, b);
                }
                else{
                    float d = 0.9*drgb[0] + 0.1*disp;
                    float k = 0.8;
                    if((r < 0.25 && g < 0.25 && b < 0.25) ||
                            (/*std::fmaxf(r, std::fmaxf(g, b)) < 0.5 &&*/
                             isNear(r, g, b, 0.05))) {
                        k = 1.0;
                    }
                    r = k*drgb[1] + (1-k)*r;
                    g = k*drgb[2] + (1-k)*g;
                    b = k*drgb[3] + (1-k)*b;
                    im3d.at<cv::Vec4f>(v, u) = cv::Vec4f(d, r, g, b);
                }
            }
        }
    }
    if(is_print) printf("Filter disp_mat done.\t");
    for (int32_t v = 0; v < im3d.rows - 1; v += step) {
        for (int32_t u = 0; u < im3d.cols - 1; u += step) {
            cv::Vec4f drgb = im3d.at<cv::Vec4f>(v, u);
            cv::Vec4f drgb_right = im3d.at<cv::Vec4f>(v, u+step);
            cv::Vec4f drgb_down = im3d.at<cv::Vec4f>(v+step, u);
            cv::Vec4f drgb_right_down = im3d.at<cv::Vec4f>(v+step, u+step);

            float disp = drgb[0];
            if(disp < 1) continue;

            float disp_right = drgb_right[0];
            float disp_down = drgb_down[0];
            float disp_right_down = drgb_right_down[0];

            //bool condition = abs(disp - gauss.mu) <= 2*gauss.sigma;
            //bool condition = disp > 1;

            if (isNear(disp, disp_right, disp_right_down)) {
                addVertex(u, v, drgb);
                addVertex(u + step, v, drgb_right);
                addVertex(u + step, v + step, drgb_right_down);
            }
            if (isNear(disp, disp_down, disp_right_down)) {
                addVertex(u, v, drgb);
                addVertex(u + step, v + step, drgb_right_down);
                addVertex(u, v + step, drgb_down);
            }
        }
    }
    if(is_print) printf("Vectices.size: %zu\n", vertices.size());

    _vavbebo->bind(&vertices[0].position.x, vertices.size() * sizeof(VertexC));
    // position attribute
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture attribute
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = vertices.size();
}
