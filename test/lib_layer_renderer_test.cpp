#include <lib_layer_renderer.h>

glm::mat4 model = glm::mat4(
            -0.583328604698181,0.418140619993210,-0.696342468261719,0.000000000000000,
            0.811959207057953,0.322681397199631,-0.486420124769211,0.000000000000000,
            0.021304894238710,-0.849142253398895,-0.527739405632019,0.000000000000000,
            0.000000000000000,0.000000000000000,37.500000000000000,1.000000000000000);


int main(int argc, char* argv[])
{
    // --------------------------- Check inputs -----------------------------
    if(argc < 3)
    {
        printf("==== lib_layer_renderer examples ===="
               "\nRun the examples as\n"
               "\t$ lib_layer_renderer_examples <LayerRenderModel> <index>\n"
               "  <LayerRenderModel> -"
               "\n\t0 - For left view"
               "\n\t1 - For right view"
               "\n\t2 - For stereo view\n"
               "  <index> - To specify the object to be rendered:"
               "\n\t1 - Render circle with coordinate"
               "\n\t2 - Render cylinder with coordinate"
               "\n\t3 - Render segment with coordinate"
               "\n\t4 - Render cone with coordinate"
               "\n\t5 - Render sphere with coordinate"
               "\n\t10 - Render gripper - needle holder"
               "\n\t11 - Render gripper - tissue grasping forceps"
               "\n\t12 - Render gripper - bipolar grasping forceps"
               "\n\tOtherValue - Render coordinate system"
               "\n");
        return 0;
    }

    using namespace mlayer;

    LayerRenderMode mode = LayerRenderMode(std::stoi(argv[1]));
    gl_util::Projection gl_proj(1120, 960, 540, 1920, 1080, 0.2, 150);
    LayerRenderer renderer(gl_proj, mode);
    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));

    if(mode == LAYER_RENDER_LEFT) view[3][0] += 2.f;
    else if(mode == LAYER_RENDER_RIGHT) view[3][0] -= 2.f;
    renderer.setView(view);
    if (mode == LAYER_RENDER_STEREO) {
        view[3][0] += 2.f;
        renderer.setView(view, LAYER_RENDER_LEFT);
        view[3][0] -= 4.f;
        renderer.setView(view, LAYER_RENDER_RIGHT);
    }
    renderer.setGlobal(model);

    // Create object
    LayerModel::Ptr layer_obj = nullptr;
    LayerModel::Ptr layer_coord = nullptr;
    glm::mat4 pose(1.f);
    // --------------------------- Prase inputs -----------------------------
    layer_coord = std::make_shared<LayerCoordinate>(10, 0.2, pose);
    unsigned char type = std::stoi(argv[2]);
    switch(type){
    case 1:
        printf("Example - Render circle with coordinate\n");
        layer_obj = std::make_shared<LayerCircle>(
                    10, glm::vec3(0.3, 0.0, 1.0), pose);
        break;
    case 2:
        printf("Example - Render cylinder with coordinate\n");
        layer_obj = std::make_shared<LayerCylinder>(
                    8, 3, glm::vec3(1.0, 0.0, 1.0), pose);
        break;
    case 3:
        printf("Example - Render segment with coordinate\n");
        layer_obj = std::make_shared<LayerSegment>(
                    20, 2, 0.4, 3, glm::vec3(1.0, 1.0, 0.0));
        break;
    case 4:
        printf("Example - Render cone with coordinate\n");
        layer_obj = std::make_shared<LayerCone>(
                    8, 3, glm::vec3(1.0, 0.3, 0.0), pose);
        break;
    case 5:
        printf("Example - Render sphere with coordinate\n");
        layer_obj = std::make_shared<LayerSphere>(
                    5, glm::vec3(0.0, 0.3, 8.0), pose);
        break;
    case 10:
        printf("Example - Render gripper - needle holder\n");
        layer_obj = std::make_shared<LayerGripper>(
                    glm::vec3(0.1, 0.8, 1.0), GRIPPER_NEEDLE_HOLDER);
        break;
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
        printf("Example - Render LayerCoordinate\n");
        break;
    }

    renderer.addLayers(layer_coord);
    renderer.addLayers(layer_obj);
    renderer.render();

    return 0;
}
