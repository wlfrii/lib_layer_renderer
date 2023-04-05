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
 * @file 		layer_coordinate.h
 * 
 * @brief 		Designed for rendering coordinate system
 * 
 * @author		Longfei Wang
 * 
 * @date		2023/01/19
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
#ifndef LAYER_COORDINATE_H_LF
#define LAYER_COORDINATE_H_LF
#include "layer_model.h"

namespace mlayer{

/**
 * @brief A class for rendering coordinate system
 */
class LayerCoordinate : public LayerModel
{
public:
    using Ptr = std::shared_ptr<LayerCoordinate>;


    /**
     * @brief Constructor of class LayerCone.
     * @param length  The length of the coordinate axis
     * @param radius  The radius of the coordinate axis
     * @param pose    The pose of the coordinate
     */
    LayerCoordinate(float length, float radius,
                    const glm::mat4& pose = glm::mat4(1.0f));


    ~LayerCoordinate();


protected:
    /* Draw the vertices */
    void draw() override;

private:
    void createCoordinate(float length, float radius, const glm::mat4& pose);

    gl_util::Shader*     _shader_x;
    gl_util::VAVBEBO*    _vavbo_x;
    size_t               _vert_num_x;

    gl_util::Shader*     _shader_y;
    gl_util::VAVBEBO*    _vavbo_y;
    size_t               _vert_num_y;
};

} // namespace::mlayer
#endif // LAYER_COORDINATE_H_LF
