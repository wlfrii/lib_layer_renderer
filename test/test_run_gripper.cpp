#include <layer_gripper.h>
#include <layer_background.h>
#include <gl_util.h>
#include <lib_math/lib_math.h>
#include <data_manager.h>


DataManager data_mgr(7);
constexpr uint8_t GRIP_IDX = 0;
glm::mat4 model = cvt2GlmMat4(data_mgr.init_grip_info.pose[GRIP_IDX]);
float angle = 0.0;
std::string image_name;
LayerBackground *layer_bg;


bool updateBackground();
void keyboardControlModel(GLFWwindow* window);

int main()
{
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
    LayerGripper layer_obj(color, data_mgr.init_grip_info.type[GRIP_IDX]);
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

    return 0;
}


bool updateBackground()
{
    cv::Mat left_rgb, right_rgb;
    float grip_angle[2];
    bool flag = data_mgr.readImage(left_rgb, right_rgb, grip_angle, image_name);
    printf("%s\n", image_name.c_str());
    if(flag) {
        cv::flip(left_rgb, left_rgb, 0);
        cv::flip(right_rgb, right_rgb, 0);
        LayerBackgroundData layer_bg_data;
        layer_bg_data.data[0] = left_rgb.data;
        layer_bg_data.data[1] = right_rgb.data;
        layer_bg_data.width = left_rgb.cols;
        layer_bg_data.height = left_rgb.rows;
        layer_bg_data.channels = left_rgb.channels();
        layer_bg->updateData(&layer_bg_data);
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
    }
    else if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS){
        updateBackground();
    }
    angle = std::fminf(std::fmaxf(angle, 0), 2*M_PI/3.f);
}
