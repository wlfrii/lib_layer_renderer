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
 * @file 		layer_sphere.h
 * 
 * @brief 		Designed for rendering sphere
 * 
 * @author		Longfei Wang
 * 
 * @date		2023/03/22
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
#ifndef LAYER_SPHERE_H_LF
#define LAYER_SPHERE_H_LF
#include "layer_model.h"

namespace mlayer{

/**
 * @brief A class for rendering circle
 */
class LayerSphere : public LayerModel
{
public:
    using Ptr = std::shared_ptr<LayerSphere>;


    /**
     * @brief Constructor of class LayerSphere.
     * @param radius  The radius of the sphere
     * @param color   The color of the sphere
     * @param pose    The pose of the sphere
     */
    LayerSphere(float radius, const glm::vec3& color,
              const glm::mat4& pose = glm::mat4(1.0f));


    ~LayerSphere();


    /**
     * @brief Change the property for current object.
     * @param radius  The radius of the sphere
     * @param pose    The pose of the sphere
     */
    void setProperty(float radius, const glm::mat4& pose);


    /**
     * @brief Set scale factors
     * @param xscale  For x direction
     * @param yscale  For y direction
     * @param zscale  For z direction
     */
    void setScaleFactors(float xscale, float yscale, float zscale) override;


private:
    glm::mat4 _pose;        //!< The pose of the cone
    float     _radius;      //!< The radius of the cone
};

} // namespace::mlayer
#endif // LAYER_SPHERE_H_LF
