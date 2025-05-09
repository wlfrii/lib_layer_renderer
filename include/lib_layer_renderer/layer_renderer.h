/**--------------------------------------------------------------------
 *
 *   				    Layer renderer library
 *
 * Description:
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
 * PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * @file 		layer_renderer.h
 *
 * @brief 		The base interface of the library
 *
 * @author		Longfei Wang
 *
 * @date		2023/02/24
 *
 * @license     GPLv3
 *
 * Copyright (C) 2021-Now Longfei Wang.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with lib_layer_renderer. If not, see <http://www.gnu.org/licenses/>
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

namespace mlayer{

/**
 * @brief Specify the render index, Left/Right for 2D mode or Stereo 3D mode.
 */
enum LayerRenderMode
{
    LAYER_RENDER_LEFT      = 0,
    LAYER_RENDER_RIGHT     = 1,
    LAYER_RENDER_STEREO    = 2,
};


/**
 * @brief The LayerRenderer class for rendering layers
 */
class LayerRenderer
{
public:
    struct WindowShotData {
        WindowShotData() : rgb_buffer(nullptr), width(0), height(0) {}
        float* rgb_buffer;
        uint16_t width;
        uint16_t height;
    };


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
                  uint16_t window_width = 1920,
                  uint16_t window_height = 1080,
                  bool is_window_visible = true);


    /**
     * @brief Constructor of LayerRenderer for multiple viewport
     * @param proj      The object of gl_util::Projection
     * @param n_viewport  The numer of view port in the specified window
     * @param window_width  The width of the specified window
     * @param window_height  The height of the specified window
     */
    LayerRenderer(const gl_util::Projection& proj,
                  const std::vector<LayerViewPort>& n_viewport,
                  uint16_t window_width = 1920,
                  uint16_t window_height = 1080,
                  bool is_window_visible = true);
    ~LayerRenderer();


    /**
     * @brief Set the same global transform for each viewport
     * @param model
     */
    void setGlobal(const glm::mat4& global);


    /**
     * @brief Set global transform for the specified viewport
     * @param model
     * @param viewport_idx  The index of viewport, ranged in [0, N_VIEWPORT-1]
     */
    void setGlobal(const glm::mat4& global, uint8_t viewport_idx);


    /**
     * @brief Set the same view for each viewport
     * @param view
     */
    void setView(const glm::mat4& view);


    /**
     * @brief Set view for the specified viewport
     * @param view
     * @param viewport_idx  The index of viewport, ranged in [0, N_VIEWPORT-1]
     */
    void setView(const glm::mat4& view, uint8_t viewport_idx);


    /**
     * @brief Set projection for the specified viewport
     * @param projection
     * @param viewport_idx  The index of viewport, ranged in [0, N_VIEWPORT-1]
     */
    void setProjection(const glm::mat4& projection, uint8_t viewport_idx);


    /**
     * @brief Add the save layer for each viewport
     * @param layer
     */
    void addLayer(std::shared_ptr<Layer> layer);


    /**
     * @brief Add layer for the specified viewport
     * @param layer
     * @param viewport_idx  The index of viewport, ranged in [0, N_VIEWPORT-1]
     */
    void addLayer(std::shared_ptr<Layer> layer, uint8_t viewport_idx);


    /**
     * @brief Clear all layers in the specified viewport
     * @param viewport_idx
     */
    void clearLayers(uint8_t viewport_idx);


    /**
     * @brief Set background color of the renderer object
     * @param r
     * @param g
     * @param b
     * @param a
     */
    void setBackgroundColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);


    /**
     * @brief setEnableKeyboardControl
     * @param flag  On/Off keyboard control;
     */
    void setEnableKeyboardControl(bool flag);


    /**
     * @brief Set control all of the viewports at the same time
     * @param flag  True for keyboard control all the viewports at the same time
     */
    void setKeyboardOnAllViewports(bool flag);


    /**
     * @brief Do rendering
     */
    void render();


    /**
     * @brief Refresh the buffer for the renderer
     * @return
     */
    void refresh();


    /**
     * @brief Should GLFW window close
     * @return
     */
    bool shouldClose() const;


    /**
     * @brief Get window ptr
     * @return
     */
    const std::shared_ptr<gl_util::Window> getWindowPtr() const;


    /**
     * @brief Get window shot
     */
    const WindowShotData& getWindowShot();


private:
    void init(const glm::mat4& projection);
    void keyboardControlModel(GLFWwindow* window);

    std::shared_ptr<gl_util::Window> _window; // The window for rendering

    size_t _N;                       // The number of viewport
    bool _keyboard_control;          // On/Off keyboard control
    bool _control_n_viewport;        // Keyboard control

    std::vector<LayerViewPort> _n_viewport; // N viewport
    std::vector<glm::mat4> _n_global;       // N local frames for each viewport
    std::vector<glm::mat4> _n_view;         // N local views for each viewport
    std::vector<glm::mat4> _n_projection;   // The projection matrix

    std::vector<std::vector<std::shared_ptr<Layer>>> _n_layers;

    WindowShotData _win_shot;
};

} // namespace::mlayer
#endif // LAYER_RENDERER_H_LF
