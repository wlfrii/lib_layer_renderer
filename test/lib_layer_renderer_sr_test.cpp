#include <lib_layer_renderer.h>
#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <lib_math/lib_math.h>


glm::mat4 model = glm::mat4(
            -0.013271633535624,0.137576162815094,0.990404427051544,0.000000000000000,
            -0.135777667164803,0.981070458889008,-0.138099983334541,0.000000000000000,
            -0.990651249885559,-0.136307477951050,0.005659478716552,0.000000000000000,
            10.000000000000000,20.000000000000000,60.000000000000000,1.000000000000000);
//glm::mat4 model = glm::mat4(
//            1, 0, 0, 0,
//            0, 0, 1, 0,
//            0, -1, 0, 0,
//            0.000000000000000,0.000000000000000,60.000000000000000,1.000000000000000);

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



int main(int argc, char* argv[])
{
    if(argc < 2) {
        printf("Please select a gripper.\n");
        std::exit(EXIT_FAILURE);
    }
    else if(argc == 3) {
        model[3][2] = std::stof(argv[2]);
    }

    using namespace mlayer;

    // ---------------------------- Settings --------------------------------
    float fxy = 1194.072289;
    float t = 4.049611;

    float fov_cam = 2.f*atan(sqrt(1920*1920+1080*1080) / 2 / fxy);
    float fov_ar = glm::radians(69.2);
    float fxy_ar = tan(fov_cam / 2.f) * fxy / tan(fov_ar / 2.f);

    printf("fov_cam: %.4f \t fxy    : %.3f\n", fov_cam, fxy);
    printf("fov_ar : %.4f \t fxy_ar : %.3f\n", fov_ar, fxy_ar);

    // IPD = [56, 72] mm
    float IPD = 64;

    float m_fov = tan(fov_cam / 2.f) / tan(fov_ar / 2.f);
    float m_xyz = IPD / t;
    printf("IPD: %.1f \t , mag_fov:%.3f, mag_xyz: %.3f, mag_z: %.3f\n",
           IPD, m_fov, m_xyz, m_fov * m_xyz);

    float z_sceen = 700;
    float w_sceen = 2.f*tan(fov_ar / 2) * z_sceen * 16/(sqrt(256 + 81));
    float h_sceen = 2.f*tan(fov_ar / 2) * z_sceen * 9/(sqrt(256 + 81));
    printf("z_sceen: %.1f, screen size: %.1f x %.1f\n",
           z_sceen, w_sceen, h_sceen);

    // Create renderer

    uint16_t w = 1920, h = 1080;
    LayerViewPort vp1(0, 0, w / 2, h / 2);          // AR left
    LayerViewPort vp2(w / 2, 0, w / 2, h / 2);      // AR right
    LayerViewPort vp3(0, h / 2, w / 2, h / 2);      // cam left
    LayerViewPort vp4(w / 2, h / 2, w / 2, h / 2);  // cam right
    std::vector<LayerViewPort> vps = {vp1, vp2, vp3, vp4};

    gl_util::Projection gl_proj(fxy, 960, 540, 1920, 1080, 0.2, 150);
    LayerRenderer renderer(gl_proj, vps);
    renderer.setKeyboardOnAllViewports(true);

    // ------------------------------------------------------------------------
    /* **** Below for camera view **** */

    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    glm::mat4 cam_view_l = view;    cam_view_l[3][0] += t / 2.f;
    renderer.setView(cam_view_l, 2);
    glm::mat4 cam_view_r = view;    cam_view_r[3][0] -= t / 2.f;
    renderer.setView(cam_view_r, 3);

    // Create coordinate
    LayerCoordinate::Ptr layer_coord(new LayerCoordinate(10, 0.2));
    layer_coord->setModel(model);
    renderer.addLayers(layer_coord, 2);
    renderer.addLayers(layer_coord, 3);

    // Add LayerModel
    float sphere_radius = 3;
    LayerModel::Ptr layer_obj(new LayerSphere(sphere_radius, glm::vec3(0.1,0.8,1.0)));
    layer_obj->setModel(model);
    renderer.addLayers(layer_obj, 2);
    renderer.addLayers(layer_obj, 3);

    // Add background
    std::string path = "./bg_025.bmp";
    cv::Mat I = cv::imread(path);
    cv::cvtColor(I, I, cv::COLOR_BGR2RGB);
    cv::flip(I, I, 0);
    cv::resize(I, I, cv::Size(3840, 1080));
    cv::Mat l_image = I.colRange(0, 1920).clone();
    cv::Mat r_image = I.colRange(1920, 3840).clone();

    LayerBackgroundData ldata(l_image.data, l_image.cols, l_image.rows,
                              l_image.channels());
    LayerBackgroundData rdata(r_image.data, r_image.cols, r_image.rows,
                              r_image.channels());

    LayerBackground::Ptr layer_bg_l(new LayerBackground);
    LayerBackground::Ptr layer_bg_r(new LayerBackground);
    layer_bg_l->updateData(&ldata);
    layer_bg_r->updateData(&rdata);

    renderer.addLayers(layer_bg_l, 2);
    renderer.addLayers(layer_bg_r, 3);


    // ------------------------------------------------------------------------
    /* **** Below for AR view **** */

    // Set view
    view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    //view[3][0] += IPD / 2;
    renderer.setView(view, 0);
    //view[3][0] -= IPD;
    renderer.setView(view, 1);
    // Set projection
    gl_util::Projection gl_proj2(fxy_ar, 960, 540, 1920, 1080, 1, 2400);
    renderer.setProjection(gl_proj2.mat4(), 0);
    renderer.setProjection(gl_proj2.mat4(), 1);


    glm::mat4 global(1.0);
    if(model[3][2] > 80) {
        global = glm::mat4(
                    0.983255326747894,0.000000000000000,0.182235598564148,0.000000000000000,
                    0.000000000000000,1.000000000000000,0.000000000000000,0.000000000000000,
                    -0.182235598564148,0.000000000000000,0.983255326747894,0.000000000000000,
                    -62.151615142822266,0.000000000000000,582.272338867187500,1.000000000000000);
    }

    // Add camera view in AR view    
    global = global * glm::mat4(
                0.165456965565681,0.055463396012783,-0.984657168388367,0.000000000000000,
                0.015458894893527,0.998149514198303,0.058821067214012,0.000000000000000,
                0.986096918582916,-0.024954041466117,0.164292693138123,0.000000000000000,
                -686.796875000000000,-13.570244789123535,1287.788330078125000,1);
    //gl_util::print("global", global);

    mlayer::LayerRenderer::WindowShotData data = renderer.getWindowShot();
    cv::Mat bg(data.height, data.width, CV_32FC3, data.rgb_buffer);
    l_image = bg.colRange(0, 960).rowRange(540, 1080).clone();
    l_image = im2uchar(l_image);
    r_image = bg.colRange(960, 1920).rowRange(540, 1080).clone();
    r_image = im2uchar(r_image);
    cv::flip(l_image, l_image, 0);
    cv::flip(r_image, r_image, 0);
    ldata = LayerBackgroundData(l_image.data, l_image.cols, l_image.rows,
                                l_image.channels());
    rdata = LayerBackgroundData(r_image.data, r_image.cols, r_image.rows,
                                r_image.channels());
    LayerBackground3D::Ptr layer_ar_l(new LayerBackground3D(w_sceen, h_sceen, z_sceen));
    layer_ar_l->updateData(&ldata);
    layer_ar_l->setModel(global);
    LayerBackground3D::Ptr layer_ar_r(new LayerBackground3D(w_sceen, h_sceen, z_sceen));
    layer_ar_r->updateData(&rdata);
    layer_ar_r->setModel(global);
    renderer.addLayers(layer_ar_l, 0);
    //renderer.addLayers(layer_ar_r, 1);

    // Add model in AR view
    Eigen::Vector3f pos(model[3][0], model[3][1], model[3][2]);
    mmath::CameraProjector cam_proj(fxy, 960, 540, t);
    Eigen::Vector2f l_uv = cam_proj.cvt3Dto2D(pos, mmath::cam::LEFT);
    Eigen::Vector2f r_uv = cam_proj.cvt3Dto2D(pos, mmath::cam::RIGHT);
    float delta_u = l_uv[0] - r_uv[0];
    float new_z = IPD * fxy_ar / delta_u;
    Eigen::Vector2f uv = cam_proj.cvt3Dto2D(pos);
    float new_x = (uv[0] - 960) / fxy_ar * new_z;
    float new_y = (uv[1] - 540) / fxy_ar * new_z;
    printf("new_z  : %f\n", new_z);
    printf("new_x  : %f, new_y: %f\n", new_x, new_y);
    model[3][0] = new_x;
    model[3][1] = new_y;
    model[3][2] = new_z;

    LayerModel::Ptr layer_obj2(new LayerSphere(sphere_radius, glm::vec3(0.1,0.8,1.0)));
    layer_obj2->setScaleFactors(m_xyz, m_xyz, m_xyz);
    layer_obj2->setModel(global * model);
    renderer.addLayers(layer_obj2, 0);
    //renderer.addLayers(layer_obj2, 1);

    // Add model coordinate in AR view
    LayerCoordinate::Ptr layer_coord2(new LayerCoordinate(10*m_xyz, 0.2*m_xyz));
    layer_coord2->setModel(global * model);
    renderer.addLayers(layer_coord2, 0);


    // Add global coordinate in AR view
    LayerCoordinate::Ptr layer_coord0(new LayerCoordinate(12*m_xyz, 0.5*m_xyz));
    layer_coord0->setModel(global);
    renderer.addLayers(layer_coord0, 0);
    //renderer.addLayers(layer_coord0, 1);


    // ----------------------------------------------------------------------
    LayerCoordinate::Ptr layer_coord22(new LayerCoordinate(10*m_xyz, 0.2*m_xyz));
    layer_coord22->setModel(model);
    renderer.addLayers(layer_coord22, 1);

    LayerModel::Ptr layer_obj22(new LayerSphere(sphere_radius, glm::vec3(0.1,0.8,1.0)));
    layer_obj22->setScaleFactors(m_xyz, m_xyz, m_xyz);
    layer_obj22->setModel(model);
    renderer.addLayers(layer_obj22, 1);

    cv::Mat blackbg(r_image.size(), CV_8UC3, cv::Scalar(50, 100, 20));
    rdata = LayerBackgroundData(blackbg.data, blackbg.cols, blackbg.rows,
                                blackbg.channels());
    LayerBackground::Ptr layer_bg_ar(new LayerBackground);
    layer_bg_ar->updateData(&rdata);
    renderer.addLayers(layer_bg_ar, 1);

#if 0
    data = renderer.getWindowShot();
    bg = cv::Mat(data.height, data.width, CV_32FC3, data.rgb_buffer);
    cv::flip(bg, bg, 0);
    cv::cvtColor(bg, bg, cv::COLOR_RGB2BGR);
//    cv::imshow("bg", bg);
//    cv::waitKey(0);
    cv::imwrite("depth_"+std::string(argv[2])+"mm.png", im2uchar(bg));
#endif

    renderer.render();

    return 0;
}
