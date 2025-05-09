#include <lib_layer_renderer.h>


glm::mat4 model = glm::mat4(
            0.952536165714264,0.272098600864410,-0.136535719037056,0.000000000000000,
            -0.063350304961205,0.615835130214691,0.785325109958649,0.000000000000000,
            0.297768801450729,-0.739400446414948,0.603843271732330,0.000000000000000,
            -9.799218177795410,2.354556083679199,39.070983886718750,1.000000000000000);

int main()
{
    printf("Run lib_layer_renderer multiview example.\n");

    using namespace mlayer;

    gl_util::Projection gl_proj(1120, 960, 540, 1920, 1080, 0.2, 150);
    uint16_t w = 1920, h = 1080;
    LayerViewPort vp1(0, 0, w / 2, h / 2);
    LayerViewPort vp2(w / 2, 0, w / 2, h / 2);
    LayerViewPort vp3(0, h / 2, w / 2, h / 2);
    LayerViewPort vp4(w / 2, h / 2, w / 2, h / 2);
    std::vector<LayerViewPort> vps = {vp1, vp2, vp3, vp4};

    LayerRenderer renderer(gl_proj, vps, w, h);
    renderer.setGlobal(model);

    // Create object
    std::shared_ptr<LayerModel> layer_coord = nullptr;
    glm::mat4 pose(1.f);
    layer_coord = std::make_shared<LayerCoordinate>(20, 0.2, pose);
    for(size_t i = 0; i < vps.size(); i++) {
        renderer.addLayer(layer_coord, i);
    }

    renderer.addLayer(std::make_shared<LayerCircle>(
                    10, glm::vec3(0.3, 0.0, 1.0), pose), 0);
    renderer.addLayer(std::make_shared<LayerCylinder>(
                    20, 4, glm::vec3(1.0, 0.0, 1.0), pose), 1);
    renderer.addLayer(std::make_shared<LayerSegment>(
                    30, 2, 0.2, 3, glm::vec3(1.0, 1.0, 0.0)), 2);
    renderer.addLayer(std::make_shared<LayerCone>(
                    20, 5, glm::vec3(1.0, 0.3, 0.0), pose), 3);
    renderer.render();


    LayerRenderer renderer2(gl_proj, mlayer::LAYER_RENDER_LEFT, w, h, true);
//    renderer2.setModel(model);
//    renderer2.addLayer(std::make_shared<LayerCylinder>(
//                    20, 4, glm::vec3(1.0, 0.0, 1.0), pose));
//    renderer2.render();

    return 0;
}
