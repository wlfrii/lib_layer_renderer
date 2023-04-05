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
 * @file 		layer_endeffector.h 
 * 
 * @brief 		Designed for rendering surgical gripper
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
#ifndef LAYER_ENDEFFECTOR_H_LF
#define LAYER_ENDEFFECTOR_H_LF
#include "layer_model.h"

namespace mlayer{

/**
 * @brief The GripperType enum specify the support gripper type.
 */
enum EndEffectorType
{
    END_EFFECTOR_NEEDLE_HOLDER_SIMPLIFIED = 0,
    END_EFFECTOR_NEEDLE_HOLDER            = 1,
    END_EFFECTOR_BIPOLAR_GRASPING_FORCEPS = 2,
    END_EFFECTOR_TISSUE_GRASPING_FORCEPS  = 3
};


/**
 * @brief A class for rendering surgical end-effector.
 */
class LayerEndEffector : public LayerModel
{
public:
    using Ptr = std::shared_ptr<LayerEndEffector>;


    /**
     * @brief Constructor of class LayLayerEndEffectorerGripper.
     * @param color  The color of the end-effector.
     * @param gtype  The type the surgical end-effector.
     */
    LayerEndEffector(glm::vec3 color, EndEffectorType gtype);
    ~LayerEndEffector();


    /**
     * @brief Set open angle for end-effector.
     * @param angle  The end-effector's open angle, default value is 0.
     */
    void setAngle(float angle);

protected:
    void draw() override;

private:
    // Load end-effector STL model
    bool loadModel();

public:
    const EndEffectorType end_effector_type;   //!< Current end-effector type

private:
    struct {
        float angle;        //!< The opening angle
        glm::vec3 p;        //!< The position of rotation axis
        glm::vec3 axis;     //!< The rotation axis
    }_active_part;          //!< The active part of the end-effector (if needed)

    gl_util::VAVBEBO* _vavbo_active;
    size_t            _vert_num_active;
    gl_util::VAVBEBO* _vavbo_active2;
    size_t            _vert_num_active2;

    gl_util::Shader*  _shader_ignore;
    gl_util::VAVBEBO* _vavbo_ignore;
    size_t            _vert_num_ignore;
};

} // namespace::mlayer
#endif // LAYER_ENDEFFECTOR_H_LF
