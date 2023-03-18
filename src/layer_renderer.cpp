#include "../export/lib_layer_renderer/layer_renderer.h"
#include "assertm.h"

namespace mlayer{

namespace{
const static glm::mat4 default_camera_view = glm::rotate(
            glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));

std::vector<LayerViewPort> createViewPort(uint16_t w, uint16_t h, uint8_t n)
{
    std::vector<LayerViewPort> vps;
    if(n == 1){
        vps = {LayerViewPort(w, h)};
    }
    else if(n == 2) {
        LayerViewPort viewport(w / 2, h);
        vps.push_back(viewport);
        viewport.x = w / 2;
        vps.push_back(viewport);
    }
    return vps;
}

std::string getAutoWindowName()
{
    static uint8_t count = 0;
    char name[32] = "layerRenderer";
    if(count > 0) {
        sprintf(name, "%s %d", name, count);
    }
    count++;
    return std::string(name);
}

}

LayerRenderer::LayerRenderer(
        const gl_util::Projection& proj, LayerRenderMode mode,
        uint16_t window_width, uint16_t window_height, bool is_window_visible)
    : _window(std::make_shared<gl_util::Window>(window_width, window_height,
                   getAutoWindowName(), is_window_visible))
    , _projection(proj.mat4())
    , _control_n_viewport(true)
    , _N(std::max(int(mode), 1))
{
    _n_viewport = createViewPort( window_width, window_height, _N);
    init();
}


LayerRenderer::LayerRenderer(
        const gl_util::Projection& proj,
        const std::vector<LayerViewPort>& n_viewport,
        uint16_t window_width, uint16_t window_height, bool is_window_visible)
    : _window(std::make_shared<gl_util::Window>(window_width, window_height,
                   getAutoWindowName(), is_window_visible))
    , _projection(proj.mat4())
    , _n_viewport(n_viewport)
    , _control_n_viewport(false)
    , _N(n_viewport.size())
{
    init();
}


void LayerRenderer::init()
{
    _window->enableDepthTest();

    _n_global.resize(_N);
    _n_view.resize(_N);
    for(size_t i = 0; i < _N; i++) {
        _n_global[i] = glm::mat4(1.0f);
        _n_view[i] = default_camera_view;
    }
    _n_layers.resize(_N);

    _win_shot.rgb_buffer = new float[_window->width*_window->height * 3];
    _win_shot.width = _window->width;
    _win_shot.height = _window->height;
}


LayerRenderer::~LayerRenderer()
{
    if(_win_shot.rgb_buffer) {
        delete [] _win_shot.rgb_buffer;
        _win_shot.rgb_buffer = nullptr;
    }
    _window->release();
}


void LayerRenderer::setGlobal(const glm::mat4& global)
{
    for(auto& m : _n_global) m = global;
}


void LayerRenderer::setGlobal(const glm::mat4& global, uint8_t viewport_idx)
{
    ASSERTM(viewport_idx < _N,
            "The input index of viewport is out of range [0, %zu]\n", _N - 1)
    _n_global[viewport_idx] = global;
}


void LayerRenderer::setView(const glm::mat4& view)
{
    for(auto& v : _n_view) v = view;
}


void LayerRenderer::setView(const glm::mat4& view, uint8_t viewport_idx)
{
    ASSERTM(viewport_idx < _N,
            "The input index of viewport is out of range [0, %zu]\n", _N - 1)
    _n_view[viewport_idx] = view;
}


void LayerRenderer::addLayers(std::shared_ptr<Layer> layer)
{
    for(auto& layers : _n_layers) layers.push_back(layer);
}


void LayerRenderer::addLayers(std::shared_ptr<Layer> layer, uint8_t viewport_idx)
{
    ASSERTM(viewport_idx < _N,
            "The input index of viewport is out of range [0, %zu]\n", _N - 1)
    _n_layers[viewport_idx].push_back(layer);
}


void LayerRenderer::setBackgroundColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    _window->setBackgroundColor(r, g, b, a);
}


void LayerRenderer::setKeyboardOnAllViewports(bool flag)
{
    _control_n_viewport = flag;
}


void LayerRenderer::render()
{
    while (!_window->shouldClose()) {
        _window->activate();
        _window->clear();

        keyboardControlModel(_window->ptr());

        for(size_t i = 0; i < _N; i++) {
            auto& layers = _n_layers[i];
            for(auto& layer : layers) {
                layer->setProjection(_projection);
                layer->setView(_n_view[i]);
                layer->setGlobal(_n_global[i]);
                layer->render(_n_viewport[i]);
            }
        }

        _window->refresh();
    }
}


const LayerRenderer::WindowShotData& LayerRenderer::getWindowShot()
{
    _window->activate();
    _window->clear();

    keyboardControlModel(_window->ptr());

    for(size_t i = 0; i < _N; i++) {
        auto& layers = _n_layers[i];
        for(auto& layer : layers) {
            layer->setProjection(_projection);
            layer->setView(_n_view[i]);
            layer->setGlobal(_n_global[i]);
            layer->render(_n_viewport[i]);
        }
    }
    glReadPixels(0, 0, _win_shot.width, _win_shot.height,
                 GL_RGB, GL_FLOAT, _win_shot.rgb_buffer);
    _window->refresh();

    return _win_shot;
}


void LayerRenderer::keyboardControlModel(GLFWwindow* window)
{
    if(_N == 0) return;

    static size_t n = 0;
    float step = 1.f;
    glm::mat4& pose = _n_global[n];

    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS){
        glfwSetWindowShouldClose(window, true);
    }
    else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
        pose = glm::translate(pose, glm::vec3(0.f, step, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        pose = glm::translate(pose, glm::vec3(0.f, -step, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
        pose = glm::translate(pose, glm::vec3(-step, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        pose = glm::translate(pose, glm::vec3(step, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){
        pose = glm::translate(pose, glm::vec3(0.f, 0.f, step));
    }
    else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS){
        pose = glm::translate(pose, glm::vec3(0.f, 0.f, -step));
    }
    else if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS){
        pose = glm::rotate(pose, glm::radians(-2.f), glm::vec3(1.f, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS){
        pose = glm::rotate(pose, glm::radians(2.f), glm::vec3(1.f, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS){
        pose = glm::rotate(pose, glm::radians(-2.f), glm::vec3(0.f, 1.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS){
        pose = glm::rotate(pose, glm::radians(2.f), glm::vec3(0.f, 1.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS){
        pose = glm::rotate(pose, glm::radians(-2.f), glm::vec3(0.f, 0.f, 1.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS){
        pose = glm::rotate(pose, glm::radians(2.f), glm::vec3(0.f, 0.f, 1.f));
    }
    else if(glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS){
        if(_N > 1 && _control_n_viewport == false){
            while(true){
                printf("LayerRenderer: ");
                printf("Specify a viewport index (0-%zu) to control: ", _N - 1);
                std::cin >> n;
                if(n < _N) {
                    printf("\tviewport %zu is under controled.\n", n);
                    break;
                }
            }
        }
    }
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS){
        gl_util::print("Model", pose);
    }

    if(_control_n_viewport) {
        setGlobal(pose);
    }
}

} // namespace::mlayer
