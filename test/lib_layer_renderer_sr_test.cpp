#include <lib_layer_renderer.h>
#include <opencv2/opencv.hpp>


glm::mat4 model = glm::mat4(
            -0.013271633535624,0.137576162815094,0.990404427051544,0.000000000000000,
            -0.135777667164803,0.981070458889008,-0.138099983334541,0.000000000000000,
            -0.990651249885559,-0.136307477951050,0.005659478716552,0.000000000000000,
            0.000000000000000,0.000000000000000,60.000000000000000,1.000000000000000);


int main(int argc, char* argv[])
{
    if(argc < 2) {
        printf("Please select a gripper.\n");
        std::exit(EXIT_FAILURE);
    }
    else if(argc >= 3) {
        model[3][2] = std::stof(argv[2]);
    }

    using namespace mlayer;

    uint16_t w = 1920, h = 1080;
    LayerViewPort vp1(0, 0, w / 2, h / 2);          // AR left
    LayerViewPort vp2(w / 2, 0, w / 2, h / 2);      // AR right
    LayerViewPort vp3(0, h / 2, w / 2, h / 2);      // cam left
    LayerViewPort vp4(w / 2, h / 2, w / 2, h / 2);  // cam right
    std::vector<LayerViewPort> vps = {vp1, vp2, vp3, vp4};

    gl_util::Projection gl_proj(1120, 960, 540, 1920, 1080, 0.2, 150);
    LayerRenderer renderer(gl_proj, vps);


    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += 2.f;
    renderer.setView(view, 2);
    view[3][0] -= 4.f;
    renderer.setView(view, 3);


    // Create coordinate
    glm::mat4 pose(1.f);
    LayerCoordinate::Ptr layer_coord(new LayerCoordinate(20, 0.2, pose));
    layer_coord->setModel(model);
    renderer.addLayers(layer_coord, 2);
    renderer.addLayers(layer_coord, 3);

    // --------------------------- Prase inputs -----------------------------
    LayerGripper::Ptr layer_obj = nullptr;
    unsigned char type = std::stoi(argv[1]);
    switch(type){
    case 11:
        printf("Example - Render gripper - tissue grasping forceps\n");
        layer_obj = std::make_shared<LayerGripper>(
                    glm::vec3(0.1, 0.8, 1.0), GRIPPER_TISSUE_GRASPING_FORCEPS);
        break;
    case 12:
        printf("Example - Render gripper - bipolar grasping forceps\n");
        layer_obj = std::make_shared<LayerGripper>(
                    glm::vec3(0.1, 0.8, 1.0), GRIPPER_BIPOLAR_GRASPING_FORCEPS);
        break;
    default:
        printf("Example - Render gripper - needle holder\n");
        layer_obj = std::make_shared<LayerGripper>(
                    glm::vec3(0.1, 0.8, 1.0), GRIPPER_NEEDLE_HOLDER);
        break;
    }
    layer_obj->setModel(model);
    layer_obj->setAngle(0.5);
    renderer.addLayers(layer_obj, 2);
    renderer.addLayers(layer_obj, 3);

    // Add background
    std::string path = "../data/bg_025.bmp";
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
//    layer_bg_l->setModel(model);

    renderer.addLayers(layer_bg_l, 2);
    renderer.addLayers(layer_bg_r, 3);

//    mlayer::LayerRenderer::WindowShotData data = renderer.getWindowShot();
//    cv::Mat bg(data.height, data.width, CV_32FC3, data.rgb_buffer);
//    cv::cvtColor(bg, bg, cv::COLOR_RGB2BGR);
//    cv::flip(bg, bg, 0);
//    cv::imshow("bg", bg);
//    cv::waitKey(0);




    renderer.render();

    return 0;
}
