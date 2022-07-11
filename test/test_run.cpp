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

#if IDX == 0
    printf("Test layerCylinder\n");
    LayerCylinder layer_obj(width, height, LAYER_CYLINDER, LAYER_RENDER_2D,
                            glm::vec3(1.0, 1.0, 0.0), {glm::vec3(0,0,0), 10, 2});
#elif IDX == 1
    printf("Test layerSegment\n");
    LayerSegment layer_obj(width, height, LAYER_SEGMENT, LAYER_RENDER_2D,
                           glm::vec3(1.0, 1.0, 0.0), {10, 0.7, 0, 2});
#elif IDX == 2
    printf("Test layerSegment and layerCylinder\n");
    float len = 10;
    float delta = 0.2;
    float radius = 2;
    LayerSegment layer_obj(width, height, LAYER_RENDER_2D,
                           glm::vec3(1.0, 1.0, 0.0), {len, theta, delta, radius});
    mmath::Pose pose = mmath::continuum::calcSingleSegmentPose(len, theta, delta);
    LayerCylinder layer_obj2(width, height, LAYER_RENDER_2D,
                            glm::vec3(1.0, 0.0, 1.0), {glm::vec3(0,0,0), 10, 2});
#endif

    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += 2.f;
    LayerModel::setView(view, 0);
    view[3][0] -= 4.f;
    LayerModel::setView(view, 1);
    gl_util::Projection gl_proj(1120, 960, 540, width, height, 0.2, 150);
    LayerModel::setProjection(gl_proj.mat4());

    // Add background
    std::string name = "../data/sequences2/frame_0001.png";
    cv::Mat image = cv::imread(name);
    cv::resize(image, image, cv::Size(3840, 1080));
    cv::Mat left = image.colRange(0, 1920).clone();
    cv::Mat right = image.colRange(1920, 3840).clone();
    cv::cvtColor(left, left, cv::COLOR_BGR2RGB);
    cv::cvtColor(right, right, cv::COLOR_BGR2RGB);
    LayerBackground layer_bg = LayerBackground(width, height, LAYER_RENDER_2D);
    LayerBackgroundData layer_bg_data(LAYER_RENDER_2D);
    layer_bg_data.data[0] = left.data;
    //layer_bg_data.data[1] = r_temp.data;
    layer_bg_data.width = left.cols;
    layer_bg_data.height = left.rows;
    layer_bg_data.channels = left.channels();
    layer_bg.updateData(&layer_bg_data);


    while (!window.shouldClose()) {
        window.activate();
        window.clear();
        glClearDepth(1.0);

        layer_bg.render(LAYER_RENDER_LEFT);

        layer_obj.setProperty({len, theta, delta, radius});
        layer_obj.setModel(model);
        layer_obj.render(LAYER_RENDER_LEFT);
        pose = mmath::continuum::calcSingleSegmentPose(len, theta, delta);
        layer_obj2.setModel(model*cvt2GlmMat4(pose));
        layer_obj2.render(LAYER_RENDER_LEFT);
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
