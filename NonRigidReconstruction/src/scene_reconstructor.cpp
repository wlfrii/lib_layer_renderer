#include "scene_reconstructor.h"
#include "sgm/sgm.h"
#include <Eigen/Dense>
#include <lib_math.h>
#include "pcl_viewer.h"
#include "util.h"
#include "point_cloud_handler.h"


namespace {
float OUTLIERS_THRESH = 2.f;
float VERTICES_DENSITY = 1.f;

bool isNear(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
            const Eigen::Vector3f& p3, float thresh = OUTLIERS_THRESH)
{
    float d1 = (p1 - p2).norm();
    float d2 = (p1 - p3).norm();
    float d3 = (p2 - p3).norm();

    float dmax = std::fmaxf(d1, std::fmaxf(d2, d3));
    if(dmax < thresh){
        return true;
    }
    else return false;
}

}

SceneReconstructor::SceneReconstructor(const mmath::CameraProjector& cam_proj,
                                       bool is_seqc)
    : _cam_proj(cam_proj)
    , _is_seqc(is_seqc)
    , _sgm(nullptr)
    , _pyd_times(2)
    , _step(2)
{
    uint16_t w = cam_proj.cx*2;
    uint16_t h = cam_proj.cy*2;
    gl_util::Projection gl_proj(cam_proj.fxy, cam_proj.cx, cam_proj.cy,
                                w, h, 0.2, 300);

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
    calcDepthMap(l_image, r_image);
    // Filter out the valid texture position
    buildVertices();

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

    glm::mat4 model = glm::translate(glm::mat4(1.f), glm::vec3(0, 0, 30));
    _layer_renderer->setModel(model);
    _layer_renderer->render();
}


void SceneReconstructor::calcDepthMap(const cv::Mat &l_image,
                                  const cv::Mat &r_image)
{
    if(l_image.cols != r_image.cols || l_image.rows != r_image.rows ||
            l_image.channels() != r_image.channels()) {
        printf("The sizes of left image and right image is different!!!\n");
        std::abort();
    }

    cv::cvtColor(l_image, _texture, cv::COLOR_BGR2RGB);

    cv::Mat left_gray, right_gray;
    cv::cvtColor(l_image, left_gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(r_image, right_gray, cv::COLOR_RGB2GRAY);

    for(int i = 0; i < _pyd_times; i++){
        cv::pyrDown(left_gray, left_gray);
        cv::pyrDown(right_gray, right_gray);
    }
//    cv::boxFilter(left_gray, left_gray, -1, cv::Size(9,9));
//    cv::boxFilter(right_gray, right_gray, -1, cv::Size(9,9));

    if(!_sgm->match(left_gray.data, right_gray.data, _disparity)){
        printf("SGM match failed!\n");
        return;
    }

    int k_pyrup = pow(2, _pyd_times);
    _depthmap = cv::Mat(_height, _width, CV_32FC1, cv::Scalar(0.f));
    std::vector<float> depthsum(_width, 0);
    for (int32_t i = 0; i < _height; i++) {
        for (int32_t j = 0; j < _width; j++) {
            const float disp = k_pyrup * _disparity[i * _width + j];
            if (disp == SGM::FLOAT_INF || disp < 1)
                continue;

            float d = _cam_proj.t * _cam_proj.fxy / disp;
            if(d < 30 || d > 150) continue;

            depthsum[j] += d;

            _depthmap.at<float>(i, j) = d;
        }
    }
    for(size_t i = 0; i < depthsum.size(); i++) {
        if(depthsum[i] > 0) {
            _u_start = i * k_pyrup;
            break;
        }
    }
    for(size_t i = depthsum.size() - 1; i > 0; i--) {
        if(depthsum[i] > 0) {
            _u_end = i* k_pyrup;
            break;
        }
    }
    printf("valid depth range: [%d, %d]\n", _u_start, _u_end);


#if DEBUG
//    double min_disp = _width, max_disp = 0;
//    cv::minMaxIdx(disp_map, &min_disp, &max_disp);
    printf("\tDisparity range in [%f, %f].\t", min_disp, max_disp);
#endif

    cv::medianBlur(_depthmap, _depthmap, 5);
    cv::medianBlur(_depthmap, _depthmap, 5);
//    cv::boxFilter(_depthmap, _depthmap, -1, cv::Size(7, 7));
    for(int i = 0; i < _pyd_times; i++){
        cv::pyrUp(_depthmap, _depthmap);
    }

//    cv::imshow("depthmap", _depthmap);
//    cv::waitKey(0);
}


void SceneReconstructor::buildVertices()
{
    // Remove the points in the gap
    int gap = 50;

    std::vector<Eigen::Vector3f> valid_points;
    for (int32_t v = gap; v < _depthmap.rows - gap; v += _step) {
        for (int32_t u = _u_start + gap; u < _u_end - gap; u += _step) {
            float d = _depthmap.at<float>(v, u);
            if(d < 30) continue;

            Eigen::Vector3f pt = _cam_proj.cvt2Dto3D(u, v, d, mmath::cam::LEFT);
            valid_points.push_back(pt);
        }
    }
    printf("Point size: %zu\n", valid_points.size());

    PointCloudHandler pthandler(valid_points);
    pthandler.voxelDownSampling(VERTICES_DENSITY);
    pthandler.rmOutliersByRadius(VERTICES_DENSITY * 4, 13);
    pthandler.rmOutliersByRadius(VERTICES_DENSITY * 2.5, 7);
    pthandler.rmOutliersByKNeighbors(7, 3);
    pcl::PolygonMesh mesh;
    pthandler.createMesh(mesh, 2.5, 2.5, 20);


    // Retrive points color
    const PointCloudXYZ& points = pthandler.getCurrentPointCloud();
    std::vector<Eigen::Vector3f> points_color(points.rows());
    for(long i = 0; i < points.rows(); i++) {
        auto& pt = points.row(i);

        Eigen::Vector2f pt2d = _cam_proj.cvt3Dto2D(pt, mmath::cam::LEFT);
        int u = round(pt2d[0]);
        int v = round(pt2d[1]);
        const cv::Vec3b pixel = _texture.at<cv::Vec3b>(v, u);
        float r = 1.f*pixel[0]/255.f;
        float g = 1.f*pixel[1]/255.f;
        float b = 1.f*pixel[2]/255.f;

        points_color[i] = {r, g, b};
    }

    // Create my vertex3D
    std::vector<mlayer::Vertex3D> layervert;
    //size_t count = 0;
    for(size_t i = 0; i < mesh.polygons.size(); i++) {
        pcl::Vertices vert = mesh.polygons[i];
        for(size_t j = 0; j < vert.vertices.size(); j++) {
            uint32_t idx = vert.vertices[j];

            auto& pt = points.row(idx);
            auto& c = points_color[idx];

            layervert.push_back({
                glm::vec4(pt[0], pt[1], pt[2], 1),
                glm::vec4(c[0], c[1], c[2], 1)
            });
        }
    }
    printf("mesh size: %zu\n", mesh.polygons.size() * 3);
    printf("layervert size: %zu\n", layervert.size());

    _layer_texture3d->updateVertex3D(layervert);
    _layer_renderer->addLayers(_layer_texture3d);

    glm::mat4 model = glm::translate(glm::mat4(1.f), glm::vec3(0, 0, 30));
    _layer_renderer->setModel(model);
    _layer_renderer->render();
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

    auto to3D = [this](int u, int v, float depth) -> Eigen::Vector3f {
        return _cam_proj.cvt2Dto3D(u, v, depth);
    };

//    cv::Mat used(_rgbd.size(), CV_8UC1, cv::Scalar(0));
//    int step = _step;
//    for (int32_t v = 0; v < _rgbd.rows - 1; v ++) {
//        for (int32_t u = 0; u < _rgbd.cols - 1; u ++) {
//            cv::Vec4f rgbd = _rgbd.at<cv::Vec4f>(v, u);
//            float disp = rgbd[3];
//            if(disp < 1) continue;

//            cv::Vec4f rgbd_right = _rgbd.at<cv::Vec4f>(v, u + step);
//            cv::Vec4f rgbd_down = _rgbd.at<cv::Vec4f>(v + step, u);
//            cv::Vec4f rgbd_right_down = _rgbd.at<cv::Vec4f>(v + step, u + step);

//            Eigen::Vector3f pt = to3D(u, v, rgbd[3]);
//            Eigen::Vector3f pt_right = to3D(u + step, v, rgbd_right[3]);
//            Eigen::Vector3f pt_down = to3D(u, v + step, rgbd_down[3]);
//            Eigen::Vector3f pt_right_down = to3D(u + step, v + step,
//                                                 rgbd_right_down[3]);

//            bool flag1 = isNear(pt, pt_right, pt_right_down);
//            bool flag2 = isNear(pt, pt_down, pt_right_down);
//            printf("flag1: %d, flag2: %d\n", flag1, flag2);
//            if (flag1) {
//                if(!_is_seqc) {
//                    addVertex(pt, rgbd);
//                    addVertex(pt_right, rgbd_right);
//                    addVertex(pt_right_down, rgbd_right_down);
//                }

//                used.at<uchar>(v, u) = 1;
//                used.at<uchar>(v, u + step) = 1;
//                used.at<uchar>(v + step, u + step) = 1;
//            }
//            if (flag2) {
//                if(!_is_seqc) {
//                    addVertex(pt, rgbd);
//                    addVertex(pt_right_down, rgbd_right_down);
//                    addVertex(pt_down, rgbd_down);
//                }

//                used.at<uchar>(v, u) = 1;
//                used.at<uchar>(v + step, u + step) = 1;
//                used.at<uchar>(v + step, u) = 1;
//            }
//        }
//    }

    // When sequence object is specified, create PCL point cloud
//    if(_is_seqc) {
//        for(int v = 0; v < used.rows - 1; v++) {
//            for(int u = 0; u < used.cols - 1; u++) {
//                uchar val = used.at<uchar>(v, u);
//                if(val == 0) continue;

//                cv::Vec4f rgbd = _rgbd.at<cv::Vec4f>(v, u);
//                //addVertex(u, v, drgb);
//            }
//        }
//    }
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

