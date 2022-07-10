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
    GRIPPER_NEEDLE_HOLDER
};


class LayerGripper : public LayerModel
{
public:
    LayerGripper(uint16_t width, uint16_t height, LayerRenderMode mode,
                 glm::vec3 color, GripperType gtype);
    ~LayerGripper();

    void setAngle(float angle);

protected:
    void draw(bool is_right) override;

private:
    bool loadModel();


public:
    const GripperType gripper_type;

private:
    struct {
        float angle;        //!< The opening angle
        glm::vec3 p;        //!< The position of rotation axis
        glm::vec3 axis;     //!< The rotation axis
    }_active_part;

    gl_util::VAVBEBO* _vavbo_active;
    size_t            _vert_num_active;

    gl_util::Shader*  _shader_ignore;
    gl_util::VAVBEBO* _vavbo_ignore;
    size_t            _vert_num_ignore;
};


#endif // LAYER_GRIPPER_H_LF
