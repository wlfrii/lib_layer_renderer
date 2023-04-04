#include <lib_layer_renderer.h>
#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <lib_math/lib_math.h>

using namespace mlayer;

// ****************************************************************************
//                                 ROI
// ****************************************************************************
struct ROI {
    ROI(uint16_t x = 0, uint16_t y = 0, uint16_t width = 0, uint16_t height = 0)
        : x(x)
        , y(y)
        , width(width)
        , height(height) {
    }
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
};

struct UIPosition3D {
    float ll_x;  // in Left image, x position of Left ui frame
    float lr_x;  // in Left image, x position of Right ui frame
    float rl_x;  // in Right image, x position of Left ui frame
    float rr_x;  // in Right image, x position of Right ui frame
};

class StereoViewer {
    struct CropInfo {
        CropInfo(float crop_ratio, float w, float h)
            : crop_ratio(crop_ratio) {
            width_ratio = w / sqrt(w * w + h * h);
            height_ratio = h / sqrt(w * w + h * h);
        }
        float crop_ratio;
        float width_ratio;
        float height_ratio;
    };

public:
    StereoViewer(float cam_fxy, float cam_t, float ar_fov);
    ~StereoViewer() {}

    /**
     * @brief getImageROIPair
     * @param work_dis  In which distance to decrease Ghosting
     * @return
     */
    std::pair<ROI, ROI> getImageROIPair(float work_dis, bool verbose = false);

    /**
     * @brief get3DUIPosition
     * @param work_dis  In which distance to decrease Ghosting
     * @param IPD  Ocular distance
     * @param z_screen  The distance between AR ocular and AR screen
     * @param z_ui2sceen  The distance between ui to AR screen
     * @param verbose  Flag for printf
     * @return
     */
    UIPosition3D get3DUIPosition(float work_dis, float IPD, float z_screen,
                                 float z_ui2sceen, bool verbose = false);

    const float cam_fxy;
    const float cam_t;
    const float ar_fov;
    const float cam_fov;
    const float ar_fxy;

private:
    CropInfo _crop_info;
};

// ****************************************************************************
//                                 main()
// ****************************************************************************
glm::mat4 model(1.f);
float z_screen = 700;
float z_ui2sceen = -50;
float work_dis = 50;


cv::Mat im2uchar(const cv::Mat &image);
std::vector<LayerModel::Ptr> createLinePair();


int main(int argc, char* argv[])
{
    if(argc < 2) {
        printf("Use default z_ui2screen is [%f]", z_ui2sceen);
    }
    else{
        z_ui2sceen = std::stof(argv[1]);
        printf("z_ui2screen is set to [%f]", z_ui2sceen);
    }
    float z_ui = z_screen + z_ui2sceen;
    printf(", so z_ui = z_screen + z_ui2sceen = [%f]\n", z_ui);

    // ------------------------------------------------------------------------
    //  Settings
    // ------------------------------------------------------------------------
    float cam_fxy = 1194.072289;
    float cam_t = 4.049611;
    float ar_fov = mmath::deg2radf(69.2);
    float IPD = 64; // IPD = [56, 72] mm

    StereoViewer sviewer(cam_fxy, cam_t, ar_fov);
    // Check image ROI
    sviewer.getImageROIPair(work_dis, true);
    // Check 3D UI ROI
    auto uipos = sviewer.get3DUIPosition(work_dis, IPD, z_screen, z_ui2sceen, true);

    // ------------------------------------------------------------------------
    //  Renderer
    // ------------------------------------------------------------------------
    uint16_t w = 1920, h = 540;
    LayerViewPort vp1(0, 0, w / 2, h);          // AR left
    LayerViewPort vp2(w / 2, 0, w / 2, h);      // AR right
    std::vector<LayerViewPort> vps = {vp1, vp2};

    gl_util::Projection gl_proj_ar(sviewer.ar_fxy, 960, 540, 1920, 1080, 0.2, 2000);
    LayerRenderer renderer(gl_proj_ar, vps, w, h);
    renderer.setBackgroundColor(255, 255, 255);
    //renderer.setKeyboardOnAllViewports(true);
    renderer.setEnableKeyboardControl(false);

    // Set view
    glm::mat4 view(1.f);
    view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += IPD / 2;
    renderer.setView(view, 0);
    view[3][0] -= IPD;
    renderer.setView(view, 1);

    // ------------------------------------------------------------------------
    //  Add line group 1
    // ------------------------------------------------------------------------
    mmath::CameraProjector ar_proj(sviewer.ar_fxy, 960, 540, IPD);

    float line_len = 50;
    float line_r   = 1;
    glm::vec3 line_c = glm::vec3(0, 0, 0);


    int ref_v = 1080 - 20;
    Eigen::Vector3f ll_pt3d = ar_proj.cvt2Dto3D(uipos.ll_x, ref_v, z_ui);
    Eigen::Vector3f lr_pt3d = ar_proj.cvt2Dto3D(uipos.lr_x, ref_v, z_ui);
    Eigen::Vector3f rl_pt3d = ar_proj.cvt2Dto3D(uipos.rl_x, ref_v, z_ui);
    Eigen::Vector3f rr_pt3d = ar_proj.cvt2Dto3D(uipos.rr_x, ref_v, z_ui);
    std::cout << "ll_pt3d: " << ll_pt3d.transpose() << std::endl;
    std::cout << "lr_pt3d: " << lr_pt3d.transpose() << std::endl;
    std::cout << "rl_pt3d: " << rl_pt3d.transpose() << std::endl;
    std::cout << "rr_pt3d: " << rr_pt3d.transpose() << std::endl;

    std::vector<Eigen::Vector3f> ref_lines(3);
    ref_lines[0] = (ll_pt3d + rl_pt3d) / 2;
    ref_lines[1] = (lr_pt3d + rr_pt3d) / 2;
    std::cout << "l_pt3d: " << ref_lines[0].transpose() << std::endl;
    std::cout << "r_pt3d: " << ref_lines[1].transpose() << std::endl;

    float line_gap = 10;
    int line_num = 4;

    ref_lines[2] = (ref_lines[0] + ref_lines[1]) / 2;
    ref_lines[2][0] += line_gap * (line_num - 1) / 2.f;
    std::cout << "c_pt3d: " << ref_lines[2].transpose() << std::endl;


    // Add bottom lines
    model = glm::mat4(1,0,0,0,  0,0,1,0,  0,-1,0,0,  0,0,0,1);
    for(size_t i = 0; i < ref_lines.size(); i++) {
        for(int j = 0; j < line_num; j++){
            LayerModel::Ptr line(new LayerCylinder(line_len, line_r, line_c));

            float k = i == 0 ? 1 : -1;
            float x_gap = k * j * line_gap;

            model[3][0] = ref_lines[i][0] + x_gap;
            model[3][1] = ref_lines[i][1];
            model[3][2] = ref_lines[i][2];

            line->setModel(model);
            renderer.addLayers(line, 0);
            renderer.addLayers(line, 1);
        }
    }
    // Add top lines
    model = glm::mat4(1,0,0,0,  0,0,-1,0,  0,1,0,0,  0,0,0,1);
    for(size_t i = 0; i < ref_lines.size(); i++) {
        for(int j = 0; j < line_num; j++){
            LayerModel::Ptr line(new LayerCylinder(line_len, line_r, line_c));

            float k = i == 0 ? 1 : -1;
            float x_gap = k * j * line_gap;

            model[3][0] = ref_lines[i][0] + x_gap;
            model[3][1] = -ref_lines[i][1];
            model[3][2] = ref_lines[i][2];

            line->setModel(model);
            renderer.addLayers(line, 0);
            renderer.addLayers(line, 1);
        }
    }


    // ------------------------------------------------------------------------
    //  Add line group 2
    // ------------------------------------------------------------------------
    std::vector<Eigen::Vector3f> h_lines(2);
    h_lines[0] = Eigen::Vector3f(ref_lines[0][0], 0, z_ui);
    h_lines[1] = Eigen::Vector3f(ref_lines[1][0], 0, z_ui);

    for(size_t i = 0; i < h_lines.size(); i++) {
        for(float j = -(line_num - 1) / 2.f; j < line_num / 2; j++){
            LayerModel::Ptr line(new LayerCylinder(line_len, line_r, line_c));

            float k = i == 0 ? 1 : -1;
            model = glm::mat4(0,0,-k,0,  0,1,0,0,  k,0,0,0,  0,0,0,1);

            float y_gap = j * line_gap;
            model[3][0] = h_lines[i][0];
            model[3][1] = h_lines[i][1] + y_gap;
            model[3][2] = h_lines[i][2];

            line->setModel(model);
            renderer.addLayers(line, 0);
            renderer.addLayers(line, 1);
        }
    }


    // ------------------------------------------------------------------------
    //  Add line group 3
    // ------------------------------------------------------------------------
    for(size_t i = 0; i < 2; i++) {
        for(float j = -(line_num - 1) / 2.f; j < line_num / 2; j++){
            LayerModel::Ptr line(new LayerCylinder(line_len, line_r, line_c));

            if(i == 0) { // vertical
                model = glm::mat4(1,0,0,0,  0,0,1,0,  0,-1,0,0,  0,0,0,1);
                float x_gap = j * line_gap;
                model[3][0] = x_gap;
                model[3][1] = line_len / 2.f;
                model[3][2] = z_ui;
            }
            else{ // horizental
                model = glm::mat4(0,0,-1,0,  0,1,0,0,  1,0,0,0,  0,0,0,1);
                float y_gap = j * line_gap;
                model[3][0] = - line_len / 2.f;
                model[3][1] = y_gap;
                model[3][2] = z_ui;
            }

            line->setModel(model);
            renderer.addLayers(line, 0);
            renderer.addLayers(line, 1);
        }
    }


    // ----------------------------------------------------------------------


#if 1
    LayerRenderer::WindowShotData data = renderer.getWindowShot();
    cv::Mat bg = cv::Mat(data.height, data.width, CV_32FC3, data.rgb_buffer);
    cv::flip(bg, bg, 0);
    cv::cvtColor(bg, bg, cv::COLOR_RGB2BGR);

    cv::imshow("bg", bg);

    char imoutname[64];
    sprintf(imoutname, "wd_%d_z_screen_%d_z_ui_%d.png",
            (int)work_dis, (int)z_screen, (int)z_ui);
    cv::resize(bg, bg, cv::Size(data.width*2, data.height*2));
    cv::imwrite(std::string(imoutname), im2uchar(bg));

    cv::waitKey(0);
#else
    renderer.render();
#endif

    return 0;
}


// ****************************************************************************
//                                Utilities
// ****************************************************************************

cv::Mat im2uchar(const cv::Mat &image)
{
    cv::Mat out;
    if(image.type() == CV_32FC1 || image.type() == CV_64FC1)
        out = cv::Mat(image.size(), CV_8UC1);
    else if(image.type() == CV_32FC3 || image.type() == CV_64FC3)
        out = cv::Mat(image.size(), CV_8UC3);

    if(image.depth() == CV_32F)
    {
        for(int i = 0; i < image.rows; i++){
            const float* row_data_in = image.ptr<float>(i);
            uchar* row_data_out = out.ptr<uchar>(i);
            for(int j = 0; j < image.cols * image.channels(); j++){
                row_data_out[j] = (uchar)MIN(row_data_in[j] * 255.0, 255);
            }
        }
    }
    else if(image.depth() == CV_64F)
    {
        for(int i = 0; i < image.rows; i++){
            const double* row_data_in = image.ptr<double>(i);
            uchar* row_data_out = out.ptr<uchar>(i);
            for(int j = 0; j < image.cols * image.channels(); j++){
                row_data_out[j] = (uchar)MIN(row_data_in[j] * 255.0, 255);
            }
        }
    }

    return out;
}


std::vector<LayerModel::Ptr> createLinePair()
{
    LayerModel::Ptr line1(new LayerCylinder(20,1,glm::vec3(0,0,0)));

    return {line1};
}


// ****************************************************************************
//                                 StereoViewer
// ****************************************************************************

StereoViewer::StereoViewer(float cam_fxy, float cam_t, float ar_fov)
    : cam_fxy(cam_fxy)
    , cam_t(cam_t)
    , ar_fov(ar_fov)
    , cam_fov(2.f*atan(sqrt(960*960 + 540*540) / cam_fxy))
    , ar_fxy(tan(cam_fov/2.f) * cam_fxy / tan(ar_fov/2.f))
    , _crop_info(StereoViewer::CropInfo(1.f - 12.f / 16, 12.f, 9))
    //        , _crop_info(StereoROI::CropInfo(1.f - 13.5f / 16, 13.5f, 9))
{
    printf("StereoViewer: Initilized done.\n");
    printf("    cam_fov: %.4f    cam_fxy: %.3f    cam_t : %.3f\n",
           cam_fov, cam_fxy, cam_t);
    printf("    ar_fov : %.4f    ar_fxy : %.3f\n",
           ar_fov, ar_fxy);
}


std::pair<ROI, ROI> StereoViewer::getImageROIPair(float work_dis, bool verbose) {
    float w_z = 2 * tanf(cam_fov / 2) * work_dis * 16 / sqrtf(256 + 81);
    float c = _crop_info.crop_ratio;
    float c1 = c / 2 + cam_t / (2*w_z);
    float c2 = c / 2 - cam_t / (2*w_z);

    auto maxi = [](int a, int b) -> int { return a >= b ? a : b; };
    auto mini = [](int a, int b) -> int { return a <= b ? a : b; };

    uint16_t lx = maxi(round(1920 * c1), 0);
    uint16_t rx = maxi(round(1920 * c2), 0);
    uint16_t w = mini(round(1920 * (1 - c)), 1920);

    std::pair<ROI, ROI> roi_pair = {ROI(lx, 0, w, 1080), ROI(rx, 0, w, 1080)};

    if(verbose) {
        printf("StereoViewer: When working distance is set to [%f]\n", work_dis);
        printf("    left roi:  [%d, %d, %d, %d], x_range:[%d, %d]\n",
               roi_pair.first.x, roi_pair.first.y,
               roi_pair.first.width, roi_pair.first.height,
               roi_pair.first.x, roi_pair.first.x + roi_pair.first.width);
        printf("    right roi: [%d, %d, %d, %d], x_range:[%d, %d]\n",
               roi_pair.second.x, roi_pair.second.y,
               roi_pair.second.width, roi_pair.second.height,
               roi_pair.second.x, roi_pair.second.x + roi_pair.second.width);
    }

    return roi_pair;
}


UIPosition3D StereoViewer::get3DUIPosition(float work_dis, float IPD,
        float z_screen, float z_ui2sceen, bool verbose) {
    /* The previous method is as follows, which is not correct.
         * Calculate the perceived depth of the screen
         *    float z_screen = IPD / (_c1 - _c2) * ar_fxy / 1920;
         * Where
         *    IPD / (_c1 - _c2) = IPD / cam_t * width_of_image_in_z_plane
         * where "IPD / cam_t" is just a coefficient for CAM->AR enlarging.
         * Thus, the calculated result "z_screen" is just an enlarged plane
         * corresponding to "image_in_z_plane". And IS NOT a real AR sceen plane
         */

    // Print the magnification
    if(verbose){
        float mag_fov = tan(cam_fov / 2.f) / tan(ar_fov / 2.f);
        float mag_xyz = IPD / cam_t;
//        printf("StereoViewer: When IPD is set to: [%f]\n", IPD);
//        printf("    mag_fov:%.3f, mag_xyz: %.3f, mag_z: %.3f\n",
//               mag_fov, mag_xyz, mag_fov * mag_xyz);
    }

    // Calculate the screen size in the known depth
    float w_screen = 2.f*tan(ar_fov / 2) * z_screen * 16/(sqrt(256 + 81));
    if(verbose) {
        float h_screen = 2.f*tan(ar_fov / 2) * z_screen * 9/(sqrt(256 + 81));
        printf("StereoViewer: When z_sceen is to: [%f]\n", z_screen);
        printf("    screen size: %.1f x %.1f\n", w_screen, h_screen);
    }

    // Calculate disparity
    float disparity = (z_ui2sceen / (z_screen + z_ui2sceen) * IPD) /
            w_screen * 1920;
    // The gap between UI and image boundary
    int gap = 10;//ceil(1.2 * abs(disparity));

    float w_z = 2 * tanf(cam_fov / 2) * work_dis * 16 / sqrtf(256 + 81);
    float c = _crop_info.crop_ratio;
    float c1 = c / 2 + cam_t / (2*w_z);
    float c2 = c / 2 - cam_t / (2*w_z);

    UIPosition3D uipos;

    uipos.ll_x = 1920 * c1 + gap;
    uipos.rl_x = uipos.ll_x - disparity;

    uipos.rr_x = 1920 * (1 - c1) - gap;
    uipos.lr_x = uipos.rr_x + disparity;

    if(verbose) {
        printf("StereoViewer: When z_sceen is to: [%f], z_ui2sceen is set: [%f]\n",
               z_screen, z_ui2sceen);
        printf("    disparity: %f\n", disparity);
        printf("    roi gap: %d\n", gap);

        printf("    left ui : [%.2f, %.2f], lr distance: [%.2f] \n",
               uipos.ll_x, uipos.lr_x, uipos.lr_x - uipos.ll_x);
        printf("    right ui: [%.2f, %.2f], lr distance: [%.2f] \n",
               uipos.rl_x, uipos.rr_x, uipos.rr_x - uipos.rl_x);
    }

    return uipos;
}
