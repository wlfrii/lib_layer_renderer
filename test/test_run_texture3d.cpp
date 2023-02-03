#include <layer_cylinder.h>
#include <layer_segment.h>
#include <layer_background.h>
#include <layer_texture3d.h>
#include <gl_util.h>
#include <lib_math/lib_math.h>
#include <opencv2/opencv.hpp>
#include <data_manager.h>


#define IS_WRITE_IMAGE 0

glm::mat4 model = glm::mat4(0.963560,0.187968,0.190531,0.000000,
                            -0.153469,0.971296,-0.182068,0.000000,
                            -0.219275,0.146184,0.964709,0.000000,
                            17.924475,-6.661742,22.319082,1.000000);
int height = 1080;
int width = 1920;
DataManager data_mgr;
LayerTexture3D *layer_tex;

bool updateBackground();
void keyboardControlModel(GLFWwindow* window);
cv::Mat im2uchar(const cv::Mat &image);

#define IDX 2
int main()
{
    gl_util::Window window(width, height);
    window.enableDepthTest();
    window.setKeyboardEventCallBack(keyboardControlModel);
//    window.setBackgroundColor(255, 255, 255);


    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += data_mgr.t / 2.f;
    LayerModel::setView(view, 0);
    view[3][0] -= data_mgr.t;
    LayerModel::setView(view, 1);
    gl_util::Projection gl_proj(data_mgr.fxy, 960, 540, width, height, 0., 150);
    LayerModel::setProjection(gl_proj.mat4());

    LayerViewPort viewport(width, height);

    printf("Test layerTexture3d\n");
    layer_tex = nullptr;
    updateBackground();

    while(!window.shouldClose()){
        window.activate();
        window.clear();
        glClearDepth(1.0);
        layer_tex->setModel(model);
        layer_tex->render(LayerViewPort(width, height), LAYER_RENDER_LEFT);

        window.refresh();

#if IS_WRITE_IMAGE
        cv::imwrite("./results_texture3d/frame_"+std::string(idx)+".png", im2uchar(bg));
#endif
    }

    window.release();
    if(layer_tex) delete layer_tex;

    return 0;
}

bool updateBackground()
{
    DataManager::FrameData fdata;
    bool flag = data_mgr.readFrame(fdata);
    printf("-----------------\n%s\n", fdata.image_name.c_str());
    for(int i = 0; i < fdata.left_mask.rows; i++) {
        for(int j = 0; j < fdata.left_mask.cols; j++) {
            uchar val = fdata.left_mask.at<uchar>(i, j);
            if(val != 0) {
                fdata.left_rgb.at<cv::Vec3b>(i, j+160) = cv::Vec3b(0, 0);
            }
        }
    }

    if(flag) {
        if(!layer_tex) {
//            layer_tex = new LayerTexture3D(left, right, data_mgr.fxy, 960, 540, 3, 1);
            layer_tex = new LayerTexture3D(
                        fdata.left_rgb, fdata.right_rgb, data_mgr.fxy, 960, 540, 3, 0);
        }
        else{
            layer_tex->update3DTexture(fdata.left_rgb, fdata.right_rgb);
        }
    }
    return flag;
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
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS){
        gl_util::print("Model", gl_util::transpose(model));
    }
    else if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS){
        updateBackground();
    }
}


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
