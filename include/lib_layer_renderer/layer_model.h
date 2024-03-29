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
 * @file 		layer_model.h 
 * 
 * @brief 		Base class for 3D object
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
#ifndef LAYER_MODEL_H_LF
#define LAYER_MODEL_H_LF
#include "layer.h"

namespace mlayer{

/**
 * @brief A base class for rendering model that requires Model, View, and
 * Projection matrix.
 */
class LayerModel : public Layer
{
protected:
    /**
     * @brief Protected constructor of class LayerModel.
     * @param type  Type of the layer.
     * @param color Color of the model
     */
    LayerModel(LayerType type, const glm::vec3 &color);

public:
    using Ptr = std::shared_ptr<LayerModel>;


    virtual ~LayerModel();


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


    /**
     * @brief Change the object color
     * @param color
     */
    void setColor(const glm::vec3 &object_color);


    /**
     * @brief Set isotropic scale factor
     * @param scale  For x, y, and z direction
     */
    void setScaleFactors(float scale);


    /**
     * @brief Set scale factors
     * @param xscale  For x direction
     * @param yscale  For y direction
     * @param zscale  For z direction
     */
    virtual void setScaleFactors(float xscale, float yscale, float zscale);



protected:
    /* Draw the vertices */
    virtual void draw() override;

    size_t     _vert_num;               //!< The number of vertex

    glm::vec3  _object_color;           //!< Color of the model
    glm::vec3  _light_color;            //!< Color of the light source
    glm::vec3  _light_pos;              //!< Position of the light source
};

} // namespace::mlayer
#endif // LAYER_MODEL_H_LF
