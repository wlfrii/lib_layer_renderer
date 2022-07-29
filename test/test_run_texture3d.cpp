#include <layer_cylinder.h>
#include <layer_segment.h>
#include <layer_background.h>
#include <layer_texture3d.h>
#include <gl_util.h>
#include <lib_math/lib_math.h>
#include <opencv2/opencv.hpp>

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

#define IS_WRITE_IMAGE 0

glm::mat4 model = glm::mat4(0.963560,0.187968,0.190531,0.000000,
                            -0.153469,0.971296,-0.182068,0.000000,
                            -0.219275,0.146184,0.964709,0.000000,
                            17.924475,-6.661742,22.319082,1.000000);
//glm::mat4 model = glm::mat4(0.963560,0.187968,0.190531,0.000000,
//                            -0.153469,0.971296,-0.182068,0.000000,
//                            -0.219275,0.146184,0.964709,0.000000,
//                            -16.501663,16.289192,173.778748,1.000000);
float theta = 0.7;
int height = 1080;
int width = 1920;

void keyboardControlModel(GLFWwindow* window);

#define IDX 2
int main()
{
    gl_util::Window window(width, height);
    window.enableDepthTest();
    window.setKeyboardEventCallBack(keyboardControlModel);
    window.setBackgroundColor(255, 255, 255);

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
    printf("Test layerTexture3d\n");
    std::string name = "../data/sequences2/frame_0001.png";
    cv::Mat image = cv::imread(name);
    cv::resize(image, image, cv::Size(3840, 1080));
    cv::Mat left = image.colRange(0, 1920).clone();
    cv::Mat right = image.colRange(1920, 3840).clone();
    cv::cvtColor(left, left, cv::COLOR_BGR2RGB);
    cv::cvtColor(right, right, cv::COLOR_BGR2RGB);
    LayerTexture3D layer_tex(left, right, 1120, 960, 540, 3);
#endif

    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += 2.f;
    LayerModel::setView(view, 0);
    view[3][0] -= 4.f;
    LayerModel::setView(view, 1);
    gl_util::Projection gl_proj(1120, 960, 540, width, height, 0., 150);
    LayerModel::setProjection(gl_proj.mat4());
    //layer_tex.setModel(model);
    LayerViewPort viewport(width, height);

#if 0
    while (!window.shouldClose()) {
        window.activate();
        window.clear();
        glClearDepth(1.0);

        layer_tex.setModel(model);
        layer_tex.render(LayerViewPort(width, height), LAYER_RENDER_LEFT);
        window.refresh();
    }
#else
    // Add background
    float* gl_color_buffer = (float*)malloc(width * height * 3 * sizeof(float));
    auto getBackground = [&window, &layer_tex, gl_color_buffer]() -> cv::Mat {
        window.activate();
        window.clear();
        glClearDepth(1.0);

        layer_tex.setModel(model);
        layer_tex.render(LayerViewPort(width, height), LAYER_RENDER_LEFT);

        glReadPixels(0, 0, width, height, GL_RGB, GL_FLOAT, gl_color_buffer);
        window.refresh();

        cv::Mat bg(height, width, CV_32FC3, gl_color_buffer);
        cv::cvtColor(bg, bg, cv::COLOR_RGB2BGR);
        cv::flip(bg, bg, 0);

        return bg;
    };

    int i = 1;
    while(true){
        char idx[5];
        sprintf(idx, "%04d", i);

        cv::Mat bg = getBackground();
        cv::imshow("Background", bg);
#if IS_WRITE_IMAGE
        cv::imwrite("./results_texture3d/frame_"+std::string(idx)+".png", im2uchar(bg));
#endif
        cv::waitKey(100);

        if(++i > 499){
            break;
        }

        std::string name = "../data/sequences2/frame_" + std::string(idx) + ".png";
        printf("%d. Image %s\n", i, idx);
        cv::Mat image = cv::imread(name);
        cv::resize(image, image, cv::Size(3840, 1080));
        cv::Mat left = image.colRange(0, 1920).clone();
        cv::Mat right = image.colRange(1920, 3840).clone();
        cv::cvtColor(left, left, cv::COLOR_BGR2RGB);
        cv::cvtColor(right, right, cv::COLOR_BGR2RGB);
        layer_tex.update3DTexture(left, right);
    }

    free(gl_color_buffer);
#endif
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