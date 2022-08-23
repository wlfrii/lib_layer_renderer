#include <layer_cylinder.h>
#include <layer_segment.h>
#include <layer_background.h>
#include <gl_util.h>
#include <lib_math/lib_math.h>
#include <opencv2/opencv.hpp>

glm::mat4 model = glm::mat4(1.000000,0.000000,0.000000,0.000000,
                  0.000000,-0.241922,-0.970296,0.000000,
                  0.000000,0.970296,-0.241922,0.000000,
                  0.000000,0.000000,57.500000,1.000000);
float theta = 0.7;

void keyboardControlModel(GLFWwindow* window);

#define IDX 2
int main()
{
    int height = 1080;
    int width = 1920;
    gl_util::Window window(width, height);
    window.enableDepthTest();
    window.setKeyboardEventCallBack(keyboardControlModel);

    float len = 10;
    float delta = 0.2;
    float radius = 2;
#if IDX == 0
    printf("Test layerCylinder\n");
    LayerCylinder layer_obj2(10, 2, glm::vec3(1.0, 0.0, 1.0));
#elif IDX == 1
    printf("Test layerSegment\n");
    LayerSegment layer_obj(len, theta, delta, radius, glm::vec3(1.0, 1.0, 0.0));
#elif IDX == 2
    printf("Test layerSegment and layerCylinder\n");
    LayerSegment layer_obj(len, theta, delta, radius, glm::vec3(1.0, 1.0, 0.0));
    mmath::Pose pose = mmath::continuum::calcSingleSegmentPose(len, theta, delta);
    LayerCylinder layer_obj2(len, radius, glm::vec3(0.0, 1.0, 1.0));
#endif

    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += 2.f;
    LayerModel::setView(view, 0);
    view[3][0] -= 4.f;
    LayerModel::setView(view, 1);
    gl_util::Projection gl_proj(1120, 960, 540, width, height, 0.2, 150);
    LayerModel::setProjection(gl_proj.mat4());

    LayerViewPort viewport(width, height);

    // Add background
    std::string name = "../data/sequences2/frame_0001.png";
    cv::Mat image = cv::imread(name);
    cv::resize(image, image, cv::Size(3840, 1080));
    cv::Mat left = image.colRange(0, 1920).clone();
    cv::Mat right = image.colRange(1920, 3840).clone();
    cv::cvtColor(left, left, cv::COLOR_BGR2RGB);
    cv::cvtColor(right, right, cv::COLOR_BGR2RGB);
    cv::flip(left, left, 0);
    cv::flip(right, right, 0);
    LayerBackground layer_bg = LayerBackground();
    LayerBackgroundData layer_bg_data;
    layer_bg_data.data[0] = left.data;
    layer_bg_data.data[1] = right.data;
    layer_bg_data.width = left.cols;
    layer_bg_data.height = left.rows;
    layer_bg_data.channels = left.channels();
    layer_bg.updateData(&layer_bg_data);
    printf("left.data:%p, bg.data:%p\n", left.data, layer_bg_data.data[0]);
    printf("right.data:%p, bg.data:%p\n", right.data, layer_bg_data.data[1]);

    auto mode = LAYER_RENDER_LEFT;
    while (!window.shouldClose()) {
        window.activate();
        window.clear();
        glClearDepth(1.0);

        //layer_bg.render(viewport, mode);

        layer_obj.setProperty(len, theta, delta);
        layer_obj.setModel(model);
        layer_obj.render(viewport, mode);
        pose = mmath::continuum::calcSingleSegmentPose(len, theta, delta);
        layer_obj2.setModel(model*cvt2GlmMat4(pose));
        layer_obj2.render(viewport, mode);
        window.refresh();
    }
    window.release();

    return 0;
}


void keyboardControlModel(GLFWwindow* window)
{
    float step = 0.5;

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
        model = glm::rotate(model, glm::radians(-2.f), glm::vec3(1.f, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(2.f), glm::vec3(1.f, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(-2.f), glm::vec3(0.f, 1.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(2.f), glm::vec3(0.f, 1.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(-2.f), glm::vec3(0.f, 0.f, 1.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(2.f), glm::vec3(0.f, 0.f, 1.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS){
        theta += 0.02;
    }
    else if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS){
        theta -= 0.02;
    }
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS){
        gl_util::print("Model", model);
    }
    theta = std::fminf(std::fmaxf(theta, 0), M_PI/2.f);
}
