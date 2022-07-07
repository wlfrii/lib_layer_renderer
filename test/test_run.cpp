#include <layer_cylinder.h>
#include <layer_segment.h>
#include <layer_util.h>
#include <gl_util.h>
#include <lib_math/lib_math.h>

glm::mat4 model = glm::mat4(1.000000,0.000000,0.000000,0.000000,
                  0.000000,-0.241922,-0.970296,0.000000,
                  0.000000,0.970296,-0.241922,0.000000,
                  0.000000,0.000000,57.500000,1.000000);

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
    float theta = 0.7;
    float delta = 0.2;
    float radius = 2;
    LayerSegment layer_obj(width, height, LAYER_SEGMENT, LAYER_RENDER_2D,
                           glm::vec3(1.0, 1.0, 0.0), {len, theta, delta, radius});
    mmath::Pose pose = mmath::continuum::calcSingleSegmentPose(len, theta, delta);
    LayerCylinder layer_obj2(width, height, LAYER_CYLINDER, LAYER_RENDER_2D,
                            glm::vec3(1.0, 0.0, 1.0), {glm::vec3(0,0,0), 10, 2});
#endif

    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += 2.f;
    LayerModelBase::setView(view, 0);
    view[3][0] -= 4.f;
    LayerModelBase::setView(view, 1);

    gl_util::Projection gl_proj(1120, 960, 540, width, height, 0.2, 150);
    LayerModelBase::setProjection(gl_proj.mat4());

    while (!window.shouldClose()) {
        window.activate();
        window.clear();
        glClearDepth(1.0);

        layer_obj.setModel(model);
        layer_obj.render(LAYER_RENDER_2D);
        layer_obj2.setModel(model*cvt2GlmMat4(pose));
        layer_obj2.render(LAYER_RENDER_2D);
        window.refresh();
    }

    window.release();

    return 0;
}


void keyboardControlModel(GLFWwindow* window)
{
    static float angle = 0;
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
        angle += 0.05;
    }
    else if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS){
        angle -= 0.05;
    }
    else if(glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS){
        gl_util::print("Model", model);
    }
    angle = std::fminf(std::fmaxf(angle, 0), M_PI/4.f);
}
