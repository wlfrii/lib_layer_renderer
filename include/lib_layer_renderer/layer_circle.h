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
 * @file 		layer_circle.h
 * 
 * @brief 		Designed for rendering circle
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
#ifndef LAYER_CIRCLE_H_LF
#define LAYER_CIRCLE_H_LF
#include "layer_model.h"

namespace mlayer{

/**
 * @brief A class for rendering circle
 */
class LayerCircle : public LayerModel
{
public:
    using Ptr = std::shared_ptr<LayerCircle>;


    /**
     * @brief Constructor of class LayerCircle.
     * @param pose    The pose of the circle
     * @param radius  The radius of the circle
     * @param color   The color of the circle
     */
    LayerCircle(float radius, const glm::vec3& color,
                const glm::mat4& pose = glm::mat4(1.0f));


    ~LayerCircle();


    /**
     * @brief Change the property for current object.
     * @param pose    The pose of the circle
     * @param radius  The radius of the circle
     */
    void setProperty(const glm::mat4& pose, float radius);


    /**
     * @brief Change the property for current object.
     * @param radius  The radius of the circle
     */
    void setProperty(float radius);


private:
    glm::mat4 _pose;        //!< The pose of the circle
    float     _radius;      //!< The radius of the circle
};

} // namespace::mlayer
#endif // LAYER_CIRCLE_H_LF
