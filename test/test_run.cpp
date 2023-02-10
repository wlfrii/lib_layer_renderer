#include <layer_circle.h>
#include <layer_cylinder.h>
#include <layer_segment.h>
#include <layer_cone.h>
#include <layer_coordinate.h>
#include <layer_background.h>
#include <layer_gripper.h>
#include <gl_util.h>
#include <lib_math/lib_math.h>
#include <opencv2/opencv.hpp>


glm::mat4 model = glm::mat4(
            1.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,
            0.000000000000000,0.984807789325714,0.173648193478584,0.000000000000000,
            0.000000000000000,-0.173648193478584,0.984807789325714,0.000000000000000,
            0.000000000000000,0.000000000000000,37.500000000000000,1.000000000000000);

float theta = 0.7;

void keyboardControlModel(GLFWwindow* window);

#define IDX 2
int main(int argc, char* argv[])
{    
    int height = 1080;
    int width = 1920;
    gl_util::Window window(width, height);
    window.enableDepthTest();
    window.setKeyboardEventCallBack(keyboardControlModel);
//    window.setBackgroundColor(0, 0, 0);


    // Create object
    LayerModel *layer_obj = nullptr;
    LayerModel *layer_coord = nullptr;
    glm::mat4 pose(1.f);
    //pose = glm::rotate(pose, glm::radians(60.f), glm::vec3(1.f, 0.f, 0.f));

    // --------------------------- Prase inputs -----------------------------
    unsigned char type = 0;
    // Check input
    if(argc < 2)
    {
        printf("Test lib render layer."
               "\nPlease input an index (0-4) to specify the object to be"
               "reendered. The deatials of the index is as follows:"
               "\n\t0 - Render coordinate system"
               "\n\t1 - Render circle with coordinate"
               "\n\t2 - Render cylinder with coordinate"
               "\n\t3 - Render segment with coordinate"
               "\n\t4 - Render cone with coordinate"
               "\n\t10 - Render gripper - needle holder"
               "\n\t11 - Render gripper - tissue grasping forceps"
               "\n\t12 - Render gripper - bipolar grasping forceps"
               "\n");
        return 0;
    }
    else{
        layer_coord = new LayerCoordinate(20, 0.2, pose);

        type = std::stoi(argv[1]);
        switch(type){
        case 1:
            printf("Test - Render circle with coordinate\n");
            layer_obj = new LayerCircle(10, glm::vec3(0.3, 0.0, 1.0), pose);
            break;
        case 2:
            printf("Test - Render cylinder with coordinate\n");
            layer_obj = new LayerCylinder(20, 4, glm::vec3(1.0, 0.0, 1.0), pose);
            break;
        case 3:
            printf("Test - Render segment with coordinate\n");
            layer_obj = new LayerSegment(30, 2, 0.2, 3, glm::vec3(1.0, 1.0, 0.0));
            break;
        case 4:
            printf("Test - Render cone with coordinate\n");
            layer_obj = new LayerCone(20, 5, glm::vec3(1.0, 0.3, 0.0), pose);
            break;
        case 10:
            printf("Test - Render gripper - needle holder\n");
            layer_obj = new LayerGripper(glm::vec3(0.1, 0.8, 1.0),
                                         GRIPPER_NEEDLE_HOLDER);
            break;
        case 11:
            printf("Test - Render gripper - tissue grasping forceps\n");
            layer_obj = new LayerGripper(glm::vec3(0.1, 0.8, 1.0),
                                         GRIPPER_TISSUE_GRASPING_FORCEPS);
            break;
        case 12:
            printf("Test - Render gripper - bipolar grasping forceps\n");
            layer_obj = new LayerGripper(glm::vec3(0.1, 0.8, 1.0),
                                         GRIPPER_BIPOLAR_GRASPING_FORCEPS);
            break;
        default:
            printf("Test - Render LayerCoordinate\n");
            break;
        }

    }
    // -------------------------------------------------------------------------
    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
    view[3][0] += 2.f;
    LayerModel::setView(view, 0);
    view[3][0] -= 4.f;
    LayerModel::setView(view, 1);
    gl_util::Projection gl_proj(1120, 960, 540, width, height, 0.2, 150);
    LayerModel::setProjection(gl_proj.mat4());

    LayerViewPort viewport(width, height);

    auto mode = LAYER_RENDER_STEREO;
    while (!window.shouldClose()) {
        window.activate();
        window.clear();
        glClearDepth(1.0);

        if(layer_coord) {
            layer_coord->setModel(model);
            layer_coord->render(viewport, mode);
        }

        if(layer_obj){
            layer_obj->setModel(model);
            layer_obj->render(viewport, mode);
        }
        window.refresh();
    }
    window.release();

    if(layer_obj){
        delete layer_obj;
        layer_obj = nullptr;
    }
    if(layer_coord){
        delete layer_coord;
        layer_coord = nullptr;
    }

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
