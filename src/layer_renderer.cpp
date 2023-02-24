#include "../include/layer_renderer.h"
#include "assertm.h"



LayerRenderer::LayerRenderer(
        const gl_util::Projection& proj,
        const std::vector<LayerViewPort>& n_viewport,
        uint16_t window_width, uint16_t window_height)
    :_window(std::make_unique<gl_util::Window>(window_width, window_height))
    , _projection(proj.mat4())
    , _n_viewport(n_viewport)
    , _N(n_viewport.size())
{
    _window->enableDepthTest();

    _n_model.resize(_N);
    _n_view.resize(_N);
    for(size_t i = 0; i < _N; i++) {
        _n_model[i] = glm::mat4(1.0f);
        _n_view[i] = glm::mat4(1.0f);
    }

    _n_layers.resize(_N);
}


void LayerRenderer::setModel(const glm::mat4& model, uint8_t viewport_idx)
{
    ASSERTM(viewport_idx < _N,
            "The input index of viewport is out of range [0, %zu]\n", _N - 1)
    _n_model[viewport_idx] = model;
}


void LayerRenderer::setView(const glm::mat4& view, uint8_t viewport_idx)
{
    ASSERTM(viewport_idx < _N,
            "The input index of viewport is out of range [0, %zu]\n", _N - 1)
    _n_view[viewport_idx] = view;
}


void LayerRenderer::setBackgroundColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    _window->setBackgroundColor(r, g, b, a);
}


void LayerRenderer::addLayers(std::shared_ptr<Layer> layer, uint8_t viewport_idx)
{
    ASSERTM(viewport_idx < _N,
            "The input index of viewport is out of range [0, %zu]\n", _N - 1)
    _n_layers[viewport_idx].push_back(layer);
}


void LayerRenderer::render()
{
    while (!_window->shouldClose()) {
        _window->activate();
        _window->clear();


        _window->refresh();
    }
}
