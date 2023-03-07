#include "scene_reconstructor.h"
#include "sgm/sgm.h"
#include <Eigen/Dense>
#include <lib_math/lib_math.h>
#include "pcl_viewer.h"
#include "util.h"
#include "point_cloud_handler.h"
#include "embedded_deformation.h"


namespace {
float VERTICES_DENSITY = 0.3f;
float MESH_SEARCH_RADIUS = VERTICES_DENSITY + 2.0;
float MESH_MU = MESH_SEARCH_RADIUS;
float MESH_NEIGHBORS = 20.f*MESH_SEARCH_RADIUS*MESH_SEARCH_RADIUS;
}


SceneReconstructor::SceneReconstructor(const mmath::CameraProjector& cam_proj,
                                       bool is_seqc)
    : _cam_proj(new mmath::CameraProjector(cam_proj))
    , _is_seqc(is_seqc)
    , _sgm(nullptr)
    , _pyd_times(2)
    , _step(2)
    , _pc_handler(new PointCloudHandler(_cam_proj, mmath::cam::LEFT))
    , _ed(nullptr)
{
    uint16_t w = cam_proj.cx*2;
    uint16_t h = cam_proj.cy*2;
    gl_util::Projection gl_proj(cam_proj.fxy, cam_proj.cx, cam_proj.cy,
                                w, h, 0.2, 300);

    mlayer::LayerViewPort vp3(0, 0, w / 2, h / 2);
    mlayer::LayerViewPort vp4(w / 2, 0, w / 2, h / 2);
    mlayer::LayerViewPort vp1(0, h / 2, w / 2, h / 2);
    mlayer::LayerViewPort vp2(w / 2, h / 2, w / 2, h / 2);
    std::vector<mlayer::LayerViewPort> vps = {vp1, vp2, vp3, vp4};
    _layer_renderer = std::make_shared<mlayer::LayerRenderer>(
                gl_proj, vps, w, h);
    glm::mat4 view = glm::rotate(
                glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += 2.f;
    _layer_renderer->setView(view, mlayer::LAYER_RENDER_LEFT);

    _layer_renderer->setKeyboardOnAllViewports(true);
    _layer_texture3d[0] = std::make_shared<mlayer::LayerTexture3D>();
    _layer_texture3d[1] = std::make_shared<mlayer::LayerTexture3D>();
    _layer_texture3d[2] = std::make_shared<mlayer::LayerTexture3D>();
    _layer_texture3d[3] = std::make_shared<mlayer::LayerTexture3D>();
    _layer_renderer->addLayers(_layer_texture3d[0], 0);
    _layer_renderer->addLayers(_layer_texture3d[1], 1);
    _layer_renderer->addLayers(_layer_texture3d[2], 2);
    _layer_renderer->addLayers(_layer_texture3d[3], 3);
}


SceneReconstructor::~SceneReconstructor()
{
    if(_sgm){
        delete _sgm;
        _sgm = nullptr;
    }
    free(_disparity);
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
        sgm_option.census_block_radius = 7;
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

    if(_ed) {
        // 测试发现，使用点云投影图做关键点匹配效果不好，这是因为点云是进过降采样后的
        // 3D位置信息，因此直接投影至2D平面后会丢失信息，即便是使用了三角化后的mesh，
        // 也存在丢失细节的问题，导致特征点匹配效果不好
//        _ed->projectPointCloud(MESH_SEARCH_RADIUS, MESH_MU,
//                               MESH_NEIGHBORS, _traingles_3d);
//        _layer_texture3d[3]->updateVertex3D(_traingles_3d);
//        cv::Mat ret = getPlotResult();
//        cv::Mat proj = ret.colRange(960, 1920).rowRange(540, 1080).clone();
//        cv::Mat prev = util::im2uchar(proj);
//        cv::resize(prev, prev, cv::Size(1920, 1080));

//        keyPointMatching(prev, l_image);
    }

    // Calculate disparity
    calcDepthMap(l_image, r_image);
    // Filter out the valid texture position
    filterPointCloud();

    // For the first image, initialize Embedded Deformation
    if(_ed == nullptr) {
        _ed = std::make_shared<EmbeddedDeformation>(4.f);
        _ed->addVertices(_pc_handler->getVertices());
    }
    else{
        // Find the correspondence between each two adjacent sequences
        std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> correspondences =
                keyPointMatching(_prev_image, l_image);

        // Update ED

    }
    _prev_image = l_image;
    _prev_depthmap = _depthmap;
}


void SceneReconstructor::plot()
{
    _layer_renderer->render();
}


cv::Mat SceneReconstructor::getPlotResult()
{
    auto data = _layer_renderer->getWindowShot();
    cv::Mat bg(data.height, data.width, CV_32FC3, data.rgb_buffer);
    cv::cvtColor(bg, bg, cv::COLOR_RGB2BGR);
    cv::flip(bg, bg, 0);

    return bg;
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

            float d = _cam_proj->t * _cam_proj->fxy / disp;
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
    printf("Valid depth range: [%d, %d]\n", _u_start, _u_end);


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


void SceneReconstructor::filterPointCloud()
{
    // Remove the points in the gap
    int gap = 50;

    pcl::PointCloud<pcl::PointXYZ>::Ptr valid_points(
                new pcl::PointCloud<pcl::PointXYZ>);
    for (int32_t v = gap; v < _depthmap.rows - gap; v += _step) {
        for (int32_t u = _u_start + gap; u < _u_end - gap; u += _step) {
            float d = _depthmap.at<float>(v, u);
            if(d < 30) continue;

            Eigen::Vector3f pt = _cam_proj->cvt2Dto3D(u, v, d, mmath::cam::LEFT);
            valid_points->push_back({pt[0], pt[1], pt[2]});
        }
    }

    _pc_handler->bindPointCloud(valid_points);
    _pc_handler->bindTexture(_texture);

    _pc_handler->createVertices(_vertices_3d);
    _layer_texture3d[0]->updateVertex(_vertices_3d);


    _pc_handler->voxelDownSampling(VERTICES_DENSITY);
    _pc_handler->rmOutliersByRadius(VERTICES_DENSITY*4, 37);
    _pc_handler->statisticalOutliersRemoval(100, 1.0f);
//    pthandler.rmOutliersByKNeighbors(20, 2);


    _pc_handler->createVertices(_vertices_3d);
    _layer_texture3d[1]->updateVertex(_vertices_3d);


    _pc_handler->createMesh(MESH_SEARCH_RADIUS, MESH_MU,
                            MESH_NEIGHBORS, _traingles_3d);
    _layer_texture3d[2]->updateVertex3D(_traingles_3d);
}


cv::Mat SceneReconstructor::EDProjection()
{
    const Vertices& _vertices = _ed->getVertices();

    std::vector<mlayer::Vertex3D> vertices3d;
    vertices3d.resize(_vertices.coords->size());
    for(size_t i = 0; i < _vertices.coords->size(); i++) {
        const pcl::PointXYZ& p = _vertices.coords->at(i);
        const Eigen::Vector3f& c = _vertices.colors[i];
        vertices3d[i] = {glm::vec4(p.x, p.y, p.z, 1),
                       glm::vec4(c[0], c[1], c[2], 1)};
    }
//    _layer_texture3d->updateVertex(vertices3d);
    auto data = _layer_renderer->getWindowShot();
    cv::Mat image(data.height, data.width, CV_32FC3, data.rgb_buffer);
    printf("image size: %dx%d\n", data.width, data.height);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    cv::flip(image, image, 0);

    return image;
}


std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>
SceneReconstructor::keyPointMatching(
        const cv::Mat &prev_image, const cv::Mat &curr_image)
{
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    int patch_size = 51;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f, 8, patch_size, 0, 2,
                                           cv::ORB::HARRIS_SCORE, patch_size);
    cv::Mat descriptors1, descriptors2;
    orb->detectAndCompute(curr_image, cv::Mat(), keypoints1, descriptors1);
    orb->detectAndCompute(prev_image, cv::Mat(), keypoints2, descriptors2);

    /* Match the two descriptors by BRMatch with Hamming */
    std::vector<cv::DMatch> matches;
    cv::BFMatcher bf_matcher(cv::NORM_HAMMING, true);
    bf_matcher.match(descriptors1, descriptors2, matches);
    printf("Found matches size: %zu\n", matches.size());

//    cv::Mat matched_image;
//    cv::drawMatches(prev_image, keypoints1, curr_image, keypoints2,
//                    matches, matched_image);
//    cv::imshow("BF Match", matched_image);
//    cv::waitKey(0);

    /* Filter the matched results */
    double min_dis = 1000, max_dis = 0;
    for (size_t i = 0; i < matches.size(); i++) {
        double dis = matches[i].distance;
        if (dis < min_dis) min_dis = dis;
        if (dis > max_dis) max_dis = dis;
    }
    printf("Match distance: [%f, %f]\n", min_dis, max_dis);

    /* Filter the matched results by 2D -> 3D */
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
                new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kd_tree->setInputCloud(_ed->getVertices().coords);
    pcl::PointXYZ query_point;

    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        // When the matches.distance > 2*min_dis, reject the match
        if (matches[i].distance > std::max(2.0 * min_dis, 30.0)) continue;

        cv::Point2i curr_pt = keypoints1[matches[i].queryIdx].pt;
        cv::Point2i prev_pt = keypoints2[matches[i].trainIdx].pt;
        if(abs(curr_pt.y - prev_pt.y) > 30) continue;

        // When depth value is too small, reject the match
        float d = _depthmap.at<float>(curr_pt);
        if(d < 30) continue;
        d = _prev_depthmap.at<float>(prev_pt);
        if(d < 30) continue;

        Eigen::Vector3f prev_pt3d = _cam_proj->cvt2Dto3D(
                    prev_pt.x, prev_pt.y, d, mmath::cam::LEFT);
        query_point.getVector3fMap() = prev_pt3d;
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        kd_tree->nearestKSearch(query_point, 1, indices, squared_distances);
        // When no corresponding 3D point is found, reject the match
        if(sqrt(squared_distances[0]) > VERTICES_DENSITY) continue;

        good_matches.push_back(matches[i]);
    }
    printf("Filtered matches size: %zu\n", good_matches.size());

//    cv::drawMatches(prev_image, keypoints1, curr_image, keypoints2,
//                    good_matches, matched_image);
//    cv::imshow("Filtered BF Match", matched_image);
//    cv::waitKey(0);


    // Retrieve the corresponding 3D point and calculate the ICP
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> correspondences;
    correspondences.resize(good_matches.size());
    for (size_t i = 0; i < good_matches.size(); i++) {
        cv::Point2i curr_pt = keypoints1[good_matches[i].queryIdx].pt;
        cv::Point2i prev_pt = keypoints2[good_matches[i].trainIdx].pt;

        float d_curr = _depthmap.at<float>(curr_pt);
        float d_prev = _prev_depthmap.at<float>(prev_pt);

        Eigen::Vector3f curr_pt3d = _cam_proj->cvt2Dto3D(
                    prev_pt.x, prev_pt.y, d_curr, mmath::cam::LEFT);
        Eigen::Vector3f prev_pt3d = _cam_proj->cvt2Dto3D(
                    prev_pt.x, prev_pt.y, d_prev, mmath::cam::LEFT);

        correspondences[i].first.getVector3fMap() = prev_pt3d;
        correspondences[i].second.getVector3fMap() = curr_pt3d;
    }

    return correspondences;
}


