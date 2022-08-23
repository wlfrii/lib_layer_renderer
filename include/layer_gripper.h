#ifndef LAYER_GRIPPER_H_LF
#define LAYER_GRIPPER_H_LF
#include "layer_model.h"

class LayerCylinder;

/**
 * @brief The GripperType enum specify the support gripper type.
 */
enum GripperType
{
    GRIPPER_NEEDLE_HOLDER_SIMPLIFIED,
    GRIPPER_NEEDLE_HOLDER,
    GRIPPER_BIPOLAR_GRASPING_FORCEPS,
    GRIPPER_TISSUE_GRASPING_FORCEPS
};


/**
 * @brief A class for rendering surgical gripper.
 */
class LayerGripper : public LayerModel
{
public:
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
    void draw(bool is_right) override;

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


#endif // LAYER_GRIPPER_H_LF
