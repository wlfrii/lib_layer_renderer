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
 * @file 		layer_gripper.h 
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
#ifndef LAYER_GRIPPER_H_LF
#define LAYER_GRIPPER_H_LF
#include "layer_model.h"

namespace mlayer{

/**
 * @brief The GripperType enum specify the support gripper type.
 */
enum GripperType
{
    GRIPPER_NEEDLE_HOLDER_SIMPLIFIED = 0,
    GRIPPER_NEEDLE_HOLDER            = 1,
    GRIPPER_BIPOLAR_GRASPING_FORCEPS = 2,
    GRIPPER_TISSUE_GRASPING_FORCEPS  = 3
};


/**
 * @brief A class for rendering surgical gripper.
 */
class LayerGripper : public LayerModel
{
public:
    using Ptr = std::shared_ptr<LayerGripper>;


    /**
     * @brief Constructor of class LayerGripper.
     * @param color  The color of the gripper.
     * @param gtype  The type the surgical gripper
     */
    LayerGripper(glm::vec3 color, GripperType gtype);
    ~LayerGripper();


    /**
     * @brief Set gripper open angle for gripper.
     * @param angle  The gripper open angle, default value is 0.
     */
    void setAngle(float angle);

protected:
    void draw() override;

private:
    // Load gripper STL model
    bool loadModel();

public:
    const GripperType gripper_type;   //!< Current gripper type

private:
    struct {
        float angle;        //!< The opening angle
        glm::vec3 p;        //!< The position of rotation axis
        glm::vec3 axis;     //!< The rotation axis
    }_active_part;          //!< The active part of the gripper (if needed)

    gl_util::VAVBEBO* _vavbo_active;
    size_t            _vert_num_active;
    gl_util::VAVBEBO* _vavbo_active2;
    size_t            _vert_num_active2;

    gl_util::Shader*  _shader_ignore;
    gl_util::VAVBEBO* _vavbo_ignore;
    size_t            _vert_num_ignore;
};

} // namespace::mlayer
#endif // LAYER_GRIPPER_H_LF
