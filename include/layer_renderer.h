/**--------------------------------------------------------------------
 *
 *   				    Layer renderer library
 *
 * Description:
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this
 * library in your scientific research work.
 *
 * @file 		layer_renderer.h
 *
 * @brief 		The base interface of the library
 *
 * @author		Longfei Wang
 *
 * @date		2023/02/24
 *
 * @license
 *
 * Copyright (C) 2021-Now Longfei Wang.
 *
 * --------------------------------------------------------------------
 * Change History:
 *
 * -------------------------------------------------------------------*/
#ifndef LAYER_RENDERER_H_LF
#define LAYER_RENDERER_H_LF
#include <gl_util.h>
#include <vector>
#include <memory>
#include "layer.h"

/**
 * @brief Specify the render index, Left/Right for 2D mode or Stereo 3D mode.
 */
enum LayerRenderMode
{
    LAYER_RENDER_LEFT   = 0,
    LAYER_RENDER_RIGHT  = 1,
    LAYER_RENDER_STEREO = 2
};


/**
 * @brief The LayerRenderer class for rendering layers
 */
class LayerRenderer
{
public:
    LayerRenderer() = delete;


    /**
     * @brief Constructor of LayerRenderer for monocular/binocular.
     * @param proj      The object of gl_util::Projection
     * @param is_binocular  Specify monocular (false) / binocular (true)
     * @param window_width  The width of the specified window
     * @param window_height  The height of the specified window
     */
    LayerRenderer(const gl_util::Projection& proj,
                  LayerRenderMode mode,
                  uint16_t window_width = 800,
                  uint16_t window_height = 600
                  );


    /**
     * @brief Constructor of LayerRenderer for multiple viewport
     * @param proj      The object of gl_util::Projection
     * @param n_viewport  The numer of view port in the specified window
     * @param window_width  The width of the specified window
     * @param window_height  The height of the specified window
     */
    LayerRenderer(const gl_util::Projection& proj,
                  const std::vector<LayerViewPort>& n_viewport,
                  uint16_t window_width = 800,
                  uint16_t window_height = 600
                  );


    /**
     * @brief Set model for each viewport
     * @param model
     * @param viewport_idx  The index of viewport, ranged in [0, N_VIEWPORT-1]
     */
    void setModel(const glm::mat4& model, uint8_t viewport_idx = 0);


    /**
     * @brief Set view for each viewport
     * @param view
     * @param viewport_idx  The index of viewport, ranged in [0, N_VIEWPORT-1]
     */
    void setView(const glm::mat4& view, uint8_t viewport_idx = 0);


    /**
     * @brief Set background color of the renderer object
     * @param r
     * @param g
     * @param b
     * @param a
     */
    void setBackgroundColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);



    void addLayers(std::shared_ptr<Layer> layer, uint8_t viewport_idx = 0);


    /**
     * @brief Do rendering
     */
    void render();


private:
    std::unique_ptr<gl_util::Window> _window; // The window for rendering

    glm::mat4 _projection;  // The projection matrix
    std::vector<LayerViewPort> _n_viewport; // N viewport

    const size_t _N;  // The number of viewport

    std::vector<glm::mat4> _n_model; // N local frames for each viewport
    std::vector<glm::mat4> _n_view;  // N local views for each viewport

    std::vector<std::vector<std::shared_ptr<Layer>>> _n_layers;
};



#endif // LAYER_RENDERER_H_LF
