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
 * @file 		layer_texture3d.h 
 * 
 * @brief 		Designed for rendering 3D texture
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
#ifndef LAYER_TEXTURE3D_H_LF
#define LAYER_TEXTURE3D_H_LF
#include "layer_model.h"

namespace mlayer{

/**
 * @brief A struct designed for vertex(position,color)
 * For now, this Type only for layer_texture3d, it might be changed in
 * the future
 */
struct Vertex3D
{
    glm::vec4 position; //!< Positon of a vertex
    glm::vec4 color;    //!< Color of a vertex
};

/**
 * @brief A class for rendering 3D texture.
 */
class LayerTexture3D : public LayerModel
{
public:
    using Ptr = std::shared_ptr<LayerTexture3D>;


    /**
     * @brief Constructor of class LayerTexture3D.
     */
    LayerTexture3D();
    ~LayerTexture3D();


    /**
     * @brief Update 3D texture of the input stereo image
     * @param 3D triangle mesh, where each adjacent 3 vertices consist of a
     * traingle
     */
    void updateVertex3D(std::vector<Vertex3D>& vertices_3d);


    void updateVertex(std::vector<Vertex3D>& vertices);

protected:
    virtual void draw() override;

    uint8_t _draw_type;

};

} // namespace::mlayer
#endif // LAYER_TEXTURE3D_H_LF
