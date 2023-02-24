#include "../include/layer_texture3d.h"
#include "sgm/sgm.h"
#include <lib_math/lib_math.h>
#include <gl_util.h>
#include "layer_define.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/brute_force.h>
#include <pcl/search/flann_search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>


namespace {
bool isNear(float d1, float d2, float d3, float thresh = 2.f)
{
    float dmax = std::fmaxf(d1, std::fmaxf(d2, d3));
    float dmin = std::fminf(d1, std::fminf(d2, d3));
    if(dmax - dmin < thresh){
        return true;
    }
    else return false;
}

cv::Mat im3d;
std::vector<VertexC> vertices;
}

LayerTexture3D::LayerTexture3D(const cv::Mat &left_tex, const cv::Mat &right_tex,
                               float fxy, float cx, float cy, float t,
                               bool is_seqc, uint8_t pyd_times)
    : LayerModel(LAYER_TEXTURE3D, glm::vec3(0.f, 0.f, 0.f))
    , _fxy(fxy), _cx(cx), _cy(cy), _t(t)
    , _is_seqc(is_seqc)
    , _sgm(new SGM())
    , _pyd_times(pyd_times)
    , _step(2)
    , _pt_cloud(new PointCloud)
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
    if(!flag) printf("LayerTexture3D initialize SGM: %d\n", flag);

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
    // Calculate disparity
    cv::Mat drgb = calcDRGB(left_tex, right_tex);
    // Filter out the valid texture position
    filterVertices(drgb);

    //printf("\tVertices.size: %zu\n", vertices.size());

    if(_is_seqc) {
        //pclProcess();
        cvProcess();
    }

    // Now, vertices are arranged triangle vertices
    _vavbebo->bind(&vertices[0].position.x, vertices.size() * sizeof(VertexC));
    // position attribute
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture attribute
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = vertices.size();
}

void LayerTexture3D::draw(bool is_right)
{
    _shader->use();
    _shader->setMat4f("projection", _projection);
    _shader->setMat4f("model", _model);
    _shader->setMat4f("view", _view[is_right]);
    auto &view = _view[is_right];
    _shader->setVec3f("view_pos", glm::vec3(view[3][0],view[3][1],view[3][2]));
    _vavbebo->bindVertexArray();
    glPointSize(2);
    if(_is_seqc){
        glDrawArrays(GL_POINTS, 0, _vert_num);
    }
    else{
        glDrawArrays(GL_TRIANGLES, 0, _vert_num);
    }
}




#define DEBUG 0

cv::Mat LayerTexture3D::calcDRGB(const cv::Mat &left_tex, const cv::Mat &right_tex)
{
    // [depth, R, G, B]
    cv::Mat drgb = cv::Mat(left_tex.size(), CV_32FC4, cv::Scalar(0, 0, 0, 0));

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
        return drgb;
    }
#if DEBUG
    printf("\tsgm match done!\n");
#endif

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
#if DEBUG
    printf("\tDisparity range in [%f, %f].\t", min_disp, max_disp);
#endif
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
#if DEBUG
    printf("xs.size: %zu, ys.size:%zu.\n", xs.size(), ys.size());
#endif
    mmath::GaussianCurvef gauss = mmath::fitGuassianCurve<float>(xs, ys);
#if DEBUG
    printf("\tGauss: mu:%f, sigma:%f, a:%f\n", gauss.mu, gauss.sigma, gauss.a);
#endif

    for(int i = 0; i < _pyd_times; i++){
        cv::pyrUp(disp_mat, disp_mat);
    }
#if DEBUG
    printf("\tdisp_mat.size: row:%d x col:%d\n", disp_mat.rows, disp_mat.cols);
    printf("\tdrgb.size: row:%d x col:%d\n", drgb.rows, drgb.cols);
#endif

    int step = _step;
    for (int32_t v = 0; v < disp_mat.rows; v += step) {
        for (int32_t u = 0; u < disp_mat.cols; u += step) {
            float disp = disp_mat.at<float>(v, u);
            bool condition = abs(disp - gauss.mu) <= 2.3*gauss.sigma;

            if (condition) {
                const cv::Vec3b pixel = _texture.at<cv::Vec3b>(v, u);
                float r = 1.f*pixel[0]/255.f;
                float g = 1.f*pixel[1]/255.f;
                float b = 1.f*pixel[2]/255.f;
                drgb.at<cv::Vec4f>(v, u) = cv::Vec4f(disp, r, g, b);

                cv::Vec4f imdrgb = im3d.at<cv::Vec4f>(v, u);
                if(imdrgb[0] < 1){
                    im3d.at<cv::Vec4f>(v, u) = cv::Vec4f(disp, r, g, b);
                }
                else{
                    float d = 0.9*imdrgb[0] + 0.1*disp;
                    float k = 0.8;
                    if((r < 0.25 && g < 0.25 && b < 0.25) ||
                            (/*std::fmaxf(r, std::fmaxf(g, b)) < 0.5 &&*/
                             isNear(r, g, b, 0.05))) {
                        k = 1.0;
                    }
                    r = k*imdrgb[1] + (1-k)*r;
                    g = k*imdrgb[2] + (1-k)*g;
                    b = k*imdrgb[3] + (1-k)*b;
                    im3d.at<cv::Vec4f>(v, u) = cv::Vec4f(d, r, g, b);
                }
            }
        }
    }

#if 0
    if(!_is_seqc) {
        printf("\tFilter again\n");
        min_disp = _width;
        max_disp = 0;
        // The outliers will be accumulated when update texture, so remove
        // outliers is added
        for (int32_t v = 0; v < im3d.rows; v += 1) {
            for (int32_t u = 0; u < im3d.cols; u += 1) {
                cv::Vec4f imdrgb = im3d.at<cv::Vec4f>(v, u);
                float d = imdrgb[0];
                if(d > 0){
                    if(d > max_disp) max_disp = d;
                    else if(d < min_disp) min_disp = d;
                }
            }
        }
        printf("\tDisparity range in [%f, %f].\t", min_disp, max_disp);
        xs = mmath::linspace<int>(floor(min_disp), 1, floor(max_disp));
        ys = std::vector<int>(xs.size(), 0);
        for (int32_t v = 0; v < im3d.rows; v += 1) {
            for (int32_t u = 0; u < im3d.cols; u += 1) {
                cv::Vec4f imdrgb = im3d.at<cv::Vec4f>(v, u);
                float d = imdrgb[0];
                if (d >= min_disp) {
                    ys[floor(d) - floor(min_disp)]++;
                }
            }
        }
        gauss = mmath::fitGuassianCurve<float>(xs, ys);
#if DEBUG
        printf("Gauss: mu:%f, sigma:%f, a:%f\n", gauss.mu, gauss.sigma, gauss.a);
#endif
        for (int32_t v = 0; v < im3d.rows; v += 1) {
            for (int32_t u = 0; u < im3d.cols; u += 1) {
                cv::Vec4f imdrgb = im3d.at<cv::Vec4f>(v, u);
                float disp = imdrgb[0];
                bool condition = abs(disp - gauss.mu) > 2*gauss.sigma;

                if (condition) {
                    im3d.at<cv::Vec4f>(v, u) = cv::Vec4f(0, 0, 0, 0);
                }
            }
        }
    }
#endif

#if DEBUG
    printf("\tFilter disp_mat done.\n");
#endif
    return _is_seqc ? drgb : im3d;
}


void LayerTexture3D::filterVertices(const cv::Mat &DRGB)
{
    vertices.clear();

    auto addVertex = [this](int u, int v,
            const cv::Vec4f &drgb) {
        float depth = this->_t * this->_fxy / drgb[0];
        float x = (u - this->_cx) / this->_fxy * depth;
        float y = (v - this->_cy) / this->_fxy * depth;

        glm::vec4 position(x, y, depth, 1.f);
        glm::vec4 color(drgb[1], drgb[2], drgb[3], 1.f);

        vertices.push_back({position, color});
    };

    cv::Mat used(DRGB.size(), CV_8UC1, cv::Scalar(0));
    int step = _step;
    for (int32_t v = 0; v < DRGB.rows - step; v += step) {
        for (int32_t u = 0; u < DRGB.cols - step; u += step) {
            cv::Vec4f drgb = DRGB.at<cv::Vec4f>(v, u);
            cv::Vec4f drgb_right = DRGB.at<cv::Vec4f>(v, u + step);
            cv::Vec4f drgb_down = DRGB.at<cv::Vec4f>(v + step, u);
            cv::Vec4f drgb_right_down = DRGB.at<cv::Vec4f>(v + step, u + step);

            float disp = drgb[0];
            if(disp < 1) continue;

            float disp_right = drgb_right[0];
            float disp_down = drgb_down[0];
            float disp_right_down = drgb_right_down[0];

            //bool condition = abs(disp - gauss.mu) <= 2*gauss.sigma;
            //bool condition = disp > 1;

            bool flag1 = isNear(disp, disp_right, disp_right_down);
            bool flag2 = isNear(disp, disp_down, disp_right_down);
            if (flag1) {
                if(!_is_seqc) {
                    addVertex(u, v, drgb);
                    addVertex(u + step, v, drgb_right);
                    addVertex(u + step, v + step, drgb_right_down);
                }

                used.at<uchar>(v, u) = 1;
                used.at<uchar>(v, u + step) = 1;
                used.at<uchar>(v + step, u + step) = 1;
            }
            if (flag2) {
                if(!_is_seqc) {
                    addVertex(u, v, drgb);
                    addVertex(u + step, v + step, drgb_right_down);
                    addVertex(u, v + step, drgb_down);
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

                cv::Vec4f drgb = DRGB.at<cv::Vec4f>(v, u);
                addVertex(u, v, drgb);
            }
        }
    }
}


// ===========================================================================
namespace {
pcl::PointXYZRGB cvt2PCLPointXYZRGB(const VertexC& vertex)
{
    glm::vec4 p = vertex.position;
    glm::vec4 c = vertex.color;
    uint8_t R = (uint8_t)MIN(c[0] * 255.f, 255);
    uint8_t G = (uint8_t)MIN(c[1] * 255.f, 255);
    uint8_t B = (uint8_t)MIN(c[2] * 255.f, 255);
    return pcl::PointXYZRGB(p[0], p[1], p[2],
            R, G, B);
}

VertexC cvt2VertexC(const pcl::PointXYZRGB& pt)
{
    glm::vec4 p(pt.x, pt.y, pt.z, 1.f);
    float R = MIN(1.f * pt.r / 255.f, 1);
    float G = MIN(1.f * pt.g / 255.f, 1);
    float B = MIN(1.f * pt.b / 255.f, 1);
    glm::vec4 c(R, G, B, 1.f);
    return {p, c};
}

VertexC cvt2VertexC(const pcl::PointXYZRGBNormal& pt)
{
    glm::vec4 p(pt.x, pt.y, pt.z, 1.f);
    float R = MIN(1.f * pt.r / 255.f, 1);
    float G = MIN(1.f * pt.g / 255.f, 1);
    float B = MIN(1.f * pt.b / 255.f, 1);
    glm::vec4 c(R, G, B, 1.f);
    return {p, c};
}

/**
 * @brief Estimate normal for the given point cloud
 * @param pt_cloud
 * @return
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr estimateNormal(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
                new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(pt_cloud);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(pt_cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(
                new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    //printf("normal.size:%zu\n", normals->size());

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(
                new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //* cloud_with_normals = cloud + normals
    pcl::concatenateFields(*pt_cloud, *normals, *cloud_with_normals);
    //printf("cloud_with_normals.size:%zu\n", cloud_with_normals->size());

    return cloud_with_normals;
}

/**
 * @brief Registering two point cloud by PCl.
 * However, the experimental results show this approach is not good.
 * @param target
 * @param source
 * @param downsample
 */
void registration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
                  bool downsample = true)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    if(downsample) {
        grid.setLeafSize(0.3, 0.3, 0.3);
        grid.setInputCloud(source);
        grid.filter(*src);
        grid.setInputCloud(target);
        grid.filter(*tgt);
    }
    else{
        src = source;
        tgt = target;
    }
    printf("ICP downsampling: target.size: %zu, source.size: %zu\n",
           tgt->size(), src->size());

#if 0 // USE Normal
    using PtType = pcl::PointXYZRGBNormal;
    pcl::PointCloud<PtType>::Ptr tgt_n = ::estimateNormal(tgt);
    pcl::PointCloud<PtType>::Ptr src_n = ::estimateNormal(src);
#else
    using PtType = pcl::PointXYZRGB;
    pcl::PointCloud<PtType>::Ptr tgt_n = tgt;
    pcl::PointCloud<PtType>::Ptr src_n = src;
#endif
    pcl::IterativeClosestPointNonLinear<PtType, PtType> icp;
    icp.setTransformationEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(2);

    icp.setInputTarget(tgt_n);
    icp.setInputSource(src_n);

    Eigen::Matrix4f src_to_tgt = Eigen::Matrix4f::Identity();
    pcl::PointCloud<PtType>::Ptr reg_result = src_n;
    for(int i = 0; i < 2; i++) {
        // Estimate
        icp.setInputSource(reg_result);
        icp.align(*reg_result);

        src_to_tgt = icp.getFinalTransformation() * src_to_tgt;
    }
    std::cout << "Src to Tgt: \n" << src_to_tgt << std::endl;
    // Transform source back in target frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*source, *output, src_to_tgt.inverse());

    pcl::registration::CorrespondenceEstimation<PtType, PtType> corres;

    corres.setInputSource(source);
    corres.setInputTarget(target);
    pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences);
    corres.determineReciprocalCorrespondences(*all_correspondences);
    printf("Correspondences (Before) : %zu\n", all_correspondences->size());


    *target += *output;
}

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
}


void LayerTexture3D::pclProcess()
{
    // Now, vertices are only storing valid locations
    // Covert the VertexC to pcl::Point
    PointCloud::Ptr pt_cloud(new PointCloud);
    for(auto& vert : ::vertices) {
        pt_cloud->emplace_back(::cvt2PCLPointXYZRGB(vert));
    }
    // Estimate normal for the PointCloud
    //PointCloudWithNormal::Ptr pt_cloud_with_normal = ::estimateNormal(pt_cloud);

    if(_pt_cloud->size() == 0) {
        _pt_cloud = pt_cloud;
    }
    else {
        // Do ICP process
        printf("ICP input: target.size: %zu, source.size: %zu\n",
               _pt_cloud->size(), pt_cloud->size());
        registration(_pt_cloud, pt_cloud);

        //PCLViewer::instance()->view(pt_cloud, "target");
    }
    printf("pt_cloud.size:%zu\n", _pt_cloud->size());

#if 0
    // The mesh calculation is time comsuming, so comment this part for now.

    pcl::search::KdTree<PointWithNormal>::Ptr tree2(
                new pcl::search::KdTree<PointWithNormal>);
//    pcl::search::FlannSearch<PointWithNormal>::Ptr tree2(
//                new pcl::search::FlannSearch<PointWithNormal>);
    tree2->setInputCloud(pt_cloud_with_normal);

    // ---- Create triangles ---
    pcl::GreedyProjectionTriangulation<PointWithNormal> rec;
    rec.setSearchRadius(1.0);    // max edge length for every triangle
    rec.setMu(2.5); // maximum acceptable distance for a point to be considered as a neighbor
    rec.setMaximumNearestNeighbors(1000);
    rec.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
    rec.setMinimumAngle(M_PI/18); // 10 degrees
    rec.setMaximumAngle(2*M_PI/3); // 120 degrees
    rec.setNormalConsistency(false);

    // Get result
    rec.setInputCloud(pt_cloud_with_normal);
    rec.setSearchMethod(tree2);

    pcl::PolygonMesh mesh;
    rec.reconstruct(mesh);
    printf("mesh.size: %zu\n", mesh.polygons.size());

    //PCLViewer::instance()->view(mesh);

    // Re-build vertices
    vertices.clear();
    for(size_t i = 0; i < mesh.polygons.size(); i++) {
        pcl::Vertices vert = mesh.polygons[i];
        for(size_t j = 0; j < vert.vertices.size(); j++) {
            uint32_t idx = vert.vertices[j];
            VertexC vert = cvt2VertexC(_pt_cloud->points[idx]);
            vertices.emplace_back(vert);
        }
    }
#else
    // Re-build vertices
    vertices.clear();
    for(size_t i = 0; i < _pt_cloud->points.size(); i++) {
        Point vert = _pt_cloud->points[i];
        vertices.emplace_back(::cvt2VertexC(vert));
    }
#endif
}


// ===========================================================================
std::vector<std::pair<cv::Point2i, cv::Point2i>>
LayerTexture3D::keyPointMatching(const cv::Mat &left_tex)
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

void LayerTexture3D::cvProcess()
{

}
