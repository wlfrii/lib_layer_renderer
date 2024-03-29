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
 * @file 		layer_cylinder.h 
 * 
 * @brief 		Designed for rendering cylinder
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
#ifndef LAYER_CYLINDER_H_LF
#define LAYER_CYLINDER_H_LF
#include "layer_model.h"

namespace mlayer{

/**
 * @brief A class for rendering cylinder
 */
class LayerCylinder : public LayerModel
{
public:
    using Ptr = std::shared_ptr<LayerCylinder>;


    /**
     * @brief Constructor of class LayerCylinder.
     * @param length  The length of the cylinder
     * @param radius  The radius of the cylinder
     * @param color   The color of the cylinder
     * @param pose  The origin of the cylinder
     */
    LayerCylinder(float length, float radius, const glm::vec3 &color,
                  const glm::mat4& pose = glm::mat4(1.f));


    ~LayerCylinder();


    /**
     * @brief Change the property for current object.
     * @param length  The length of the cylinder
     * @param radius  The radius of the cylinder
     * @param pose  The origin of the cylinder
     */
    void setProperty(float length, float radius, const glm::mat4 &pose);


    /**
     * @brief Change the property for current object.
     * @param length  The length of the cylinder
     * @param radius  The radius of the cylinder
     */
    void setProperty(float length, float radius);


    /**
     * @brief Change the property for current object.
     * @param length  The length of the cylinder
     */
    void setProperty(float length);


    /**
     * @brief Set scale factors
     * @param xscale  For x direction
     * @param yscale  For y direction
     * @param zscale  For z direction
     */
    void setScaleFactors(float xscale, float yscale, float zscale) override;


private:
    glm::mat4 _pose;      //!< The origin of the cylinder
    float     _length;    //!< The length of the cylinder
    float     _radius;    //!< The radius of the cylinder
};

} // namespace::mlayer
#endif // LAYER_CYLINDER_H_LF
