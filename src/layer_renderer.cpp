#include "../include/layer_renderer.h"
#include "assertm.h"

namespace{
const static glm::mat4 default_camera_view = glm::rotate(
            glm::mat4(1.0), glm::radians(180.f), glm::vec3(1.f,0.f,0.f));
}

LayerRenderer::LayerRenderer(
        const gl_util::Projection& proj, LayerRenderMode mode,
        uint16_t window_width, uint16_t window_height)
    :_window(std::make_unique<gl_util::Window>(window_width, window_height))
    , _projection(proj.mat4())
    , _N(std::max(int(mode), 1))
    , _mode(mode)
{
    _window->enableDepthTest();

    if(_N == 1) _n_viewport = {LayerViewPort(window_width, window_height)};
    else{
        LayerViewPort viewport(window_width / 2, window_height);
        _n_viewport.push_back(viewport);
        viewport.x = window_width / 2;
        _n_viewport.push_back(viewport);
    }

    _n_model.resize(_N);
    _n_view.resize(_N);
    for(size_t i = 0; i < _N; i++) {
        _n_model[i] = glm::mat4(1.0f);
        _n_view[i] = default_camera_view;
    }

    _n_layers.resize(_N);
}


LayerRenderer::LayerRenderer(
        const gl_util::Projection& proj,
        const std::vector<LayerViewPort>& n_viewport,
        uint16_t window_width, uint16_t window_height)
    :_window(std::make_unique<gl_util::Window>(window_width, window_height))
    , _projection(proj.mat4())
    , _n_viewport(n_viewport)
    , _N(n_viewport.size())
    , _mode(LAYER_RENDER_MULTIPLE)
{
    _window->enableDepthTest();

    _n_model.resize(_N);
    _n_view.resize(_N);
    for(size_t i = 0; i < _N; i++) {
        _n_model[i] = glm::mat4(1.0f);
        _n_view[i] = default_camera_view;
    }

    _n_layers.resize(_N);
}


LayerRenderer::~LayerRenderer()
{

}


void LayerRenderer::setModel(const glm::mat4& model)
{
    for(auto& m : _n_model) m = model;
}


void LayerRenderer::setModel(const glm::mat4& model, uint8_t viewport_idx)
{
    ASSERTM(viewport_idx < _N,
            "The input index of viewport is out of range [0, %zu]\n", _N - 1)
    _n_model[viewport_idx] = model;
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
                layer->setModel(_n_model[i]);
                layer->render(_n_viewport[i]);
            }
        }

        _window->refresh();
    }
}


void LayerRenderer::keyboardControlModel(GLFWwindow* window)
{
    if(_N == 0) return;

    static size_t n = 0;
    float step = 0.5;
    glm::mat4& model = _n_model[n];

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
    else if(glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS){
        if(_mode == LAYER_RENDER_MULTIPLE && _N > 1){
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
        gl_util::print("Model", model);
    }

    if(_mode == LAYER_RENDER_STEREO) {
        setModel(model);
    }
}
