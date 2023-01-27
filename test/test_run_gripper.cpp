#include <layer_gripper.h>
#include <layer_background.h>
#include <gl_util.h>
#include <lib_math/lib_math.h>
#include <data_manager.h>
#include <surgical_tool_manager.h>


DataManager data_mgr(11);
constexpr uint8_t GRIP_IDX = 0;
glm::mat4 model;// = cvt2GlmMat4(data_mgr.init_grip_info.poses[GRIP_IDX]);
float angle = 0.0;
LayerBackground *layer_bg;

SurgicalToolManager *STMgr;
//Eigen::Matrix4f T_tb_2_cam;
mmath::Pose T_tb_2_cam;

bool updateBackground();
void keyboardControlModel(GLFWwindow* window);
//void findTtb2cam();

int main()
{
    STMgr = new SurgicalToolManager();

    mmath::Pose n_model(-16,-2,38);
    model = cvt2GlmMat4(n_model);

    int height = 1080;
    int width = 1920;
    gl_util::Window window(width, height);
    window.enableDepthTest();
    window.setKeyboardEventCallBack(keyboardControlModel);

    // Set view and projection
    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += data_mgr.t / 2.f;
    LayerModel::setView(view, 0);
    view[3][0] -= data_mgr.t;
    LayerModel::setView(view, 1);
    gl_util::Projection gl_proj(data_mgr.fxy, 960, 540, width, height, 0.2, 150);
    LayerModel::setProjection(gl_proj.mat4());

    LayerViewPort viewport(width, height);

    // Intilize surgical tools
    SurgicalToolParam tool_param(80, 29.5, 19.6, 5.71, 4); // Lg = 18.9
    STMgr->initialize(TOOL1, tool_param, SURGICAL_TOOL_TYPE_SP_TOOL, 0);

    // Add gripper
    glm::vec3 color(0.f, 0.8f, 0.8f);
#define IDX 3
#if IDX == 0
    printf("Test NH\n");
    LayerGripper layer_obj(color, GRIPPER_NEEDLE_HOLDER);
#elif IDX == 1
    printf("Test BGF\n");
    LayerGripper layer_obj(color, GRIPPER_BIPOLAR_GRASPING_FORCEPS);
#elif IDX == 2
    printf("Test TGF\n");
    LayerGripper layer_obj(color, GRIPPER_TISSUE_GRASPING_FORCEPS);
#else
    LayerGripper layer_obj(color, data_mgr.init_grip_info.types[GRIP_IDX]);
#endif

    // Update background
    layer_bg = new LayerBackground();
    updateBackground();

    // Start the render loop
    auto mode = LAYER_RENDER_STEREO;
    while (!window.shouldClose()) {
        window.activate();
        window.clear();
        glClearDepth(1.0);

        layer_obj.setAngle(angle);
        layer_obj.setModel(model);
        layer_obj.render(viewport, mode);
        layer_bg->render(viewport, mode);

        window.refresh();
    }
    window.release();
    delete layer_bg;
    delete STMgr;

    return 0;
}


bool updateBackground()
{
    DataManager::FrameData fdata;
    bool flag = data_mgr.readFrame(fdata);
    printf("%s\n", fdata.image_name.c_str());
    if(flag) {
        cv::flip(fdata.left_rgb, fdata.left_rgb, 0);
        cv::flip(fdata.right_rgb, fdata.right_rgb, 0);
        LayerBackgroundData layer_bg_data;
        layer_bg_data.data[0] = fdata.left_rgb.data;
        layer_bg_data.data[1] = fdata.right_rgb.data;
        layer_bg_data.width = fdata.left_rgb.cols;
        layer_bg_data.height = fdata.left_rgb.rows;
        layer_bg_data.channels = fdata.left_rgb.channels();
        layer_bg->updateData(&layer_bg_data);

        printf("\t update config.\n");
        SurgicalToolConfig config(fdata.psi[0].L_insert + 19.6+29.5,
                                  fdata.psi[0].phi,
                                  fdata.psi[0].theta1, fdata.psi[0].delta1,
                                  fdata.psi[0].theta2, fdata.psi[0].delta2);
        STMgr->updateConfig(TOOL1, config);
        mmath::Pose T_t2e_2_tb = STMgr->getEnd2BasePose(TOOL1);

//        std::cout << "config: \n" << config << "\n";
//        std::cout << "T_t2e_2_tb: \n" << T_t2e_2_tb << "\n";

        Eigen::Matrix3f R_grip_init;
        R_grip_init << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        if(data_mgr.current_image_idx - 1 == data_mgr.min_image_idx) {
            mmath::Pose T_g_2_cam(data_mgr.init_grip_info.poses[GRIP_IDX]);
            T_g_2_cam.R *= R_grip_init.transpose();

            if(!T_g_2_cam.isUnitOrthogonal()) {
                T_g_2_cam.unitOrthogonalize();
                printf("T_g_2_cam is not unit-orthogonal.\n");
            }

            T_tb_2_cam = T_g_2_cam * T_t2e_2_tb.inverse();

//            T_tb_2_cam = data_mgr.T_tb2cam;

            std::cout << "T_tb_2_cam: \n" << T_tb_2_cam << "\n";
        }


        auto T_g_2_cam = T_tb_2_cam * T_t2e_2_tb;

        T_g_2_cam.R *= R_grip_init;

        model = cvt2GlmMat4(T_g_2_cam);
        angle = fdata.angles[0];

//        printf("config:%s\n", config.info());
//        std::cout << config << "\n";
//        std::cout << T_t2e_2_tb << "\n" << T_g_2_cam << "\n";
        printf("T_g_2_cam: %s\n", T_g_2_cam.info());
    }
    else{
        data_mgr.current_image_idx = data_mgr.min_image_idx;
    }

    return flag;
}

void keyboardControlModel(GLFWwindow* window)
{
    float step = 0.05;
    float astep = glm::radians(0.1);

    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS){
        glfwSetWindowShouldClose(window, true);
    }
    else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(0.f, step, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(0.f, -step, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(-step, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(step, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(0.f, 0.f, step));
    }
    else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(0.f, 0.f, -step));
    }
    else if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS){
        model = glm::rotate(model, -astep, glm::vec3(1.f, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS){
        model = glm::rotate(model, astep, glm::vec3(1.f, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS){
        model = glm::rotate(model, -astep, glm::vec3(0.f, 1.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS){
        model = glm::rotate(model, astep, glm::vec3(0.f, 1.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS){
        model = glm::rotate(model, -astep, glm::vec3(0.f, 0.f, 1.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS){
        model = glm::rotate(model, astep, glm::vec3(0.f, 0.f, 1.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS){
        angle += 0.005;
    }
    else if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS){
        angle -= 0.005;
    }
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS){
        printf("Gripper idx: %d angle: %f\n", GRIP_IDX, angle);
    }
    else if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS){
        gl_util::print("Model", gl_util::transpose(model));
        mmath::Pose pose = cvt2Pose(model);
        printf("pose unit-orthogonal: %d\n", pose.isUnitOrthogonal());
//        Eigen::Vector3f ypr = pose.R.eulerAngles(2, 1, 0);
//        printf("%f,%f,%f,%f,%f,%f\n", ypr[2], ypr[1], ypr[0],
//                pose.t[0], pose.t[1], pose.t[2]);
    }
    else if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS){
        updateBackground();
    }
    angle = std::fminf(std::fmaxf(angle, 0), 2*M_PI/3.f);
}


void findTtb2cam()
{
    SurgicalToolParam tool_param(80, 29.5, 19.6, 5.71, 4); // Lg = 18.9
    SurgicalToolConfig config = SurgicalToolConfig(79.058144 + 29.5 + 19.6,
                                -0.017924, 0.003210, -0.124690,
                                0.012064, 3.012736);
    STMgr->initialize(TOOL1, tool_param, SURGICAL_TOOL_TYPE_SP_TOOL, 0);
    STMgr->updateConfig(TOOL1, config);
    mmath::Pose T_2e_wrt_b = STMgr->getEnd2BasePose(TOOL1);

    // For sequence_09
    Eigen::Matrix3f R_grip_init;
    R_grip_init << 0, -1, 0,
            1, 0, 0,
            0, 0, 1;
    mmath::Pose T_g_2_cam(data_mgr.init_grip_info.poses[GRIP_IDX]);
    T_g_2_cam.R *= R_grip_init.transpose();

    if(!T_g_2_cam.isUnitOrthogonal()) {
        T_g_2_cam.unitOrthogonalize();
        printf("T_g_2_cam is not unit-orthogonal.\n");
    }

    mmath::Pose T_cam2image(mmath::rotByZf(mmath::PI));
    T_tb_2_cam = T_g_2_cam * T_2e_wrt_b.inverse();// * T_cam2image.T();
    std::cout << "T_tb_2_cam: \n" << T_tb_2_cam << "\n";
}
