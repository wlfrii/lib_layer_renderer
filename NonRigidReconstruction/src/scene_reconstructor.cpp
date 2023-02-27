#include "scene_reconstructor.h"
#include "sgm/sgm.h"
#include <Eigen/Dense>
#include <lib_math.h>


namespace {
bool isNear(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
            const Eigen::Vector3f& p3, float thresh = 2.f)
{
    float d1 = (p1 - p2).norm();
    float d2 = (p1 - p3).norm();
    float d3 = (p2 - p3).norm();

    float dmax = std::fmaxf(d1, std::fmaxf(d2, d3));
    float dmin = std::fminf(d1, std::fminf(d2, d3));
    if(dmax - dmin < thresh){
        return true;
    }
    else return false;
}

}

SceneReconstructor::SceneReconstructor(const mmath::CameraProjector& cam_proj,
                                       bool is_seqc, uint8_t pyd_times)
    : _cam_proj(cam_proj)
    , _is_seqc(is_seqc)
    , _sgm(nullptr)
    , _pyd_times(pyd_times)
    , _step(2)
{
    uint16_t w = cam_proj.cx*2;
    uint16_t h = cam_proj.cy*2;
    gl_util::Projection gl_proj(cam_proj.fxy, cam_proj.cx, cam_proj.cy,
                                w, h, 0.2, 150);

    _layer_renderer = new mlayer::LayerRenderer(
                gl_proj, mlayer::LAYER_RENDER_LEFT, w, h);

    _layer_texture3d = std::make_shared<mlayer::LayerTexture3D>();
}


SceneReconstructor::~SceneReconstructor()
{
    if(_sgm){
        delete _sgm;
        _sgm = nullptr;
    }
    free(_disparity);

    if(_layer_renderer) {
        delete _layer_renderer;
        _layer_renderer = nullptr;
    }
}


void SceneReconstructor::reconstruct(const cv::Mat &l_image,
                                     const cv::Mat &r_image)
{
    if(l_image.cols != r_image.cols || l_image.rows != r_image.rows ||
            l_image.channels() != r_image.channels()) {
        printf("The sizes of left image and right image is different!!!\n");
        std::abort();
    }

    if(!_sgm) {
        _sgm = new SGM();

        _width = l_image.cols;
        _height = l_image.rows;
        _rgbd = cv::Mat(_height, _width, CV_32FC4);

        for(int i = 0; i < _pyd_times; i++){
            _width /= 2;
            _height /= 2;
        }
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
        bool flag = _sgm->initialize(_width, _height, sgm_option);
        if(!flag) {
            printf("Initialized SGM failed\n");
            delete _sgm;
            _sgm = nullptr;
            return;
        }
        _disparity = (float*)malloc(_width * _height * sizeof(float));
    }


    // Calculate disparity
    calcRGBD(l_image, r_image);
    // Filter out the valid texture position
    filterVertices();

    //printf("\tVertices.size: %zu\n", vertices.size());

    if(_is_seqc) {
        //pclProcess();
        cvProcess();
    }
}


void SceneReconstructor::plot()
{
    _layer_texture3d->updateVertex3D(_vertices_3d);
    _layer_renderer->addLayers(_layer_texture3d);
}


#define DEBUG 0

void SceneReconstructor::calcRGBD(const cv::Mat &l_image,
                                  const cv::Mat &r_image)
{
    if(l_image.cols != r_image.cols || l_image.rows != r_image.rows ||
            l_image.channels() != r_image.channels()) {
        printf("The sizes of left image and right image is different!!!\n");
        std::abort();
    }

    _rgbd = cv::Mat(l_image.size(), CV_32FC4, cv::Scalar(0, 0, 0, 0));

    _texture = l_image;
    cv::Mat left_gray, right_gray;
    cv::cvtColor(l_image, left_gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(r_image, right_gray, cv::COLOR_RGB2GRAY);

    for(int i = 0; i < _pyd_times; i++){
        cv::pyrDown(left_gray, left_gray);
        cv::pyrDown(right_gray, right_gray);
    }
    cv::boxFilter(left_gray, left_gray, -1, cv::Size(9,9));
    cv::boxFilter(right_gray, right_gray, -1, cv::Size(9,9));

    if(!_sgm->match(left_gray.data, right_gray.data, _disparity)){
        printf("SGM match failed!\n");
        return;
    }
#if DEBUG
    printf("\tsgm match done!\n");
#endif

    int k_pyrup = pow(2, _pyd_times);
    cv::Mat disp_map = cv::Mat(_height, _width, CV_32FC1, cv::Scalar(0.f));
    for (int32_t i = 0; i < _height; i++) {
        for (int32_t j = 0; j < _width; j++) {
            const float disp = k_pyrup * _disparity[i * _width + j];
            if (disp != SGM::FLOAT_INF) {
                disp_map.at<float>(i, j) = disp;
            }
        }
    }
    double min_disp = _width, max_disp = 0;
    cv::minMaxIdx(disp_map, &min_disp, &max_disp);
#if DEBUG
    printf("\tDisparity range in [%f, %f].\t", min_disp, max_disp);
#endif

    for(int i = 0; i < _pyd_times; i++){
        cv::pyrUp(disp_map, disp_map);
    }
#if DEBUG
    printf("\tdisp_map.size: row:%d x col:%d\n", disp_map.rows, disp_map.cols);
    printf("\trgbd.size: row:%d x col:%d\n", rgbd.rows, rgbd.cols);
#endif

    int step = _step;
    for (int32_t v = 0; v < disp_map.rows; v += step) {
        for (int32_t u = 0; u < disp_map.cols; u += step) {
            float disp = disp_map.at<float>(v, u);
            const cv::Vec3b pixel = _texture.at<cv::Vec3b>(v, u);
            float r = 1.f*pixel[0]/255.f;
            float g = 1.f*pixel[1]/255.f;
            float b = 1.f*pixel[2]/255.f;
            _rgbd.at<cv::Vec4f>(v, u) = cv::Vec4f(r, g, b, disp);
        }
    }
}


void SceneReconstructor::filterVertices()
{
    _vertices_3d.clear();

    auto addVertex = [this](const Eigen::Vector3f& pt,
            const cv::Vec4f &rgbd) {
        mlayer::Vertex3D vertex3d;
        vertex3d.position = glm::vec4(pt[0], pt[1], pt[2], 1.f);
        vertex3d.color = glm::vec4(rgbd[0], rgbd[1], rgbd[2], 1.f);

        _vertices_3d.push_back(vertex3d);
    };

    auto to3D = [this](int u, int v, float disp) -> Eigen::Vector3f {
        float depth = _cam_proj.t * _cam_proj.fxy / disp;
        return _cam_proj.cvt2Dto3D(u, v, depth);
    };

    cv::Mat used(_rgbd.size(), CV_8UC1, cv::Scalar(0));
    int step = _step;
    for (int32_t v = 0; v < _rgbd.rows - step; v += step) {
        for (int32_t u = 0; u < _rgbd.cols - step; u += step) {
            cv::Vec4f rgbd = _rgbd.at<cv::Vec4f>(v, u);
            float disp = rgbd[3];
            if(disp < 1) continue;

            cv::Vec4f rgbd_right = _rgbd.at<cv::Vec4f>(v, u + step);
            cv::Vec4f rgbd_down = _rgbd.at<cv::Vec4f>(v + step, u);
            cv::Vec4f rgbd_right_down = _rgbd.at<cv::Vec4f>(v + step, u + step);

            Eigen::Vector3f pt = to3D(u, v, rgbd[3]);
            Eigen::Vector3f pt_right = to3D(u + step, v, rgbd_right[3]);
            Eigen::Vector3f pt_down = to3D(u, v + step, rgbd_down[3]);
            Eigen::Vector3f pt_right_down = to3D(u + step, v + step,
                                                 rgbd_right_down[3]);

            bool flag1 = isNear(pt, pt_right, pt_right_down);
            bool flag2 = isNear(pt, pt_down, pt_right_down);
            if (flag1) {
                if(!_is_seqc) {
                    addVertex(pt, rgbd);
                    addVertex(pt_right, rgbd_right);
                    addVertex(pt_right_down, rgbd_right_down);
                }

                used.at<uchar>(v, u) = 1;
                used.at<uchar>(v, u + step) = 1;
                used.at<uchar>(v + step, u + step) = 1;
            }
            if (flag2) {
                if(!_is_seqc) {
                    addVertex(pt, rgbd);
                    addVertex(pt_right_down, rgbd_right_down);
                    addVertex(pt_down, rgbd_down);
                }

                used.at<uchar>(v, u) = 1;
                used.at<uchar>(v + step, u + step) = 1;
                used.at<uchar>(v + step, u) = 1;
            }
        }
    }

    // When sequence object is specified, create PCL point cloud
    if(_is_seqc) {
        for(int v = 0; v < used.rows - 1; v++) {
            for(int u = 0; u < used.cols - 1; u++) {
                uchar val = used.at<uchar>(v, u);
                if(val == 0) continue;

                cv::Vec4f drgb = _rgbd.at<cv::Vec4f>(v, u);
                //addVertex(u, v, drgb);
            }
        }
    }
}




// ===========================================================================
std::vector<std::pair<cv::Point2i, cv::Point2i>>
SceneReconstructor::keyPointMatching(const cv::Mat &left_tex)
{
    cv::Mat prev_image, curr_image;
    cv::cvtColor(_texture, prev_image, cv::COLOR_RGB2BGR);
    cv::cvtColor(left_tex, curr_image, cv::COLOR_RGB2BGR);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->detect(prev_image, keypoints1);
    orb->detect(curr_image, keypoints2);

    cv::Mat descriptors1, descriptors2;
    orb->compute(prev_image, keypoints1, descriptors1);
    orb->compute(curr_image, keypoints2, descriptors2);

    // Match the BRIEF of two descriptors by BRMatch with Hamming
    std::vector<cv::DMatch> matches;
    cv::BFMatcher bf_matcher(cv::NORM_HAMMING, true);
    bf_matcher.match(descriptors1, descriptors2, matches);
    printf("Found matches size: %zu\n", matches.size());

    cv::Mat matched_image;
    cv::drawMatches(prev_image, keypoints1, curr_image, keypoints2,
                    matches, matched_image);
    cv::imshow("BF Match", matched_image);
    cv::waitKey(0);

    // Filter the matched results
    double min_dis = 1000, max_dis = 0;
    for (size_t i = 0; i < matches.size(); i++) {
        double dis = matches[i].distance;
        if (dis < min_dis) min_dis = dis;
        if (dis > max_dis) max_dis = dis;
    }
    printf("Match distance: [%f, %f]\n", min_dis, max_dis);
    // When the matches.distance > 2*min_dis, reject the match
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i].distance <= std::max(2.0 * min_dis, 20.0))
            good_matches.push_back(matches[i]);
    }
    printf("Filtered matches size: %zu\n", good_matches.size());


    cv::drawMatches(prev_image, keypoints1, curr_image, keypoints2,
                    good_matches, matched_image);
    cv::imshow("Filtered BF Match", matched_image);
    cv::waitKey(0);

    // Retrive the keypoint locations
    std::vector<std::pair<cv::Point2i, cv::Point2i>> ret;


    return ret;
}

void SceneReconstructor::cvProcess()
{

}

