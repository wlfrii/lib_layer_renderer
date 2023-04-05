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
 * @file 		layer_backgrounrd3D.h
 * 
 * @brief 		Designed for rendering 3D plane background
 * 
 * @author		Longfei Wang
 * 
 * @date		2022/04/01
 * 
 * @license		GPLv3
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
#ifndef LAYER_BACKGROUND3D_H_LF
#define LAYER_BACKGROUND3D_H_LF
#include "layer_background.h"


namespace mlayer{

/**
 * @brief A class for rendering window 3D plane background
 */
class LayerBackground3D : public LayerBackground
{
public:
    using Ptr = std::shared_ptr<LayerBackground3D>;


    LayerBackground3D(float bgwidth, float bgheight, float bgdepth);
    ~LayerBackground3D();


    /**
     * @brief Set Model matrix
     * @param model
     */
    void setModel(const glm::mat4& model) override;


    /**
     * @brief Set View matrix
     * @param view
     */
    void setView(const glm::mat4& view) override;


    /**
     * @brief Set Projection matrix
     * @param proj
     */
    void setProjection(const glm::mat4& proj) override;


protected:
    void draw() override;
};

} // namespace::mlayer
#endif // LAYER_BACKGROUND3D_H_LF
