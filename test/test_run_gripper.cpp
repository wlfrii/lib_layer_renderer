#include <layer_gripper.h>
#include <gl_util.h>
#include <lib_math/lib_math.h>


glm::mat4 model = glm::mat4(1.000000,0.000000,0.000000,0.000000,
                  0.000000,-0.241922,-0.970296,0.000000,
                  0.000000,0.970296,-0.241922,0.000000,
                  0.000000,0.000000,57.500000,1.000000);
float theta = 0.0;

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
    view[3][0] += 2.f;
    LayerModel::setView(view, 0);
    view[3][0] -= 4.f;
    LayerModel::setView(view, 1);
    gl_util::Projection gl_proj(1120, 960, 540, width, height, 0.2, 150);
    LayerModel::setProjection(gl_proj.mat4());

    LayerViewPort viewport(width, height);

    // Add gripper
    glm::vec3 color(0.7, 0.7, 0.7);
#define IDX 2
#if IDX == 0
    printf("Test NH\n");
    LayerGripper layer_obj(color, GRIPPER_NEEDLE_HOLDER);
#elif IDX == 1
    printf("Test BGF\n");
    LayerGripper layer_obj(color, GRIPPER_BIPOLAR_GRASPING_FORCEPS);
#elif IDX == 2
    printf("Test TGF\n");
    LayerGripper layer_obj(color, GRIPPER_TISSUE_GRASPING_FORCEPS);
#endif

    auto mode = LAYER_RENDER_STEREO;
    while (!window.shouldClose()) {
        window.activate();
        window.clear();
        glClearDepth(1.0);

        layer_obj.setAngle(theta);
        layer_obj.setModel(model);
        layer_obj.render(viewport, mode);

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
