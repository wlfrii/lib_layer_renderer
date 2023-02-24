# lib_layer_renderer

This module is splited from XXXX project (_which was designed as a static library mainly for rendering continuum-based surgical robot and gripper_).

Currently, this library is modularized for rendering shapes, model, vertices, and graph based on OpenGL.

## Requirements

+ [lib_math](https://github.com/wlfrii/lib_math) - which can be found in my git repository and includes some useful calculation utilities.
+ [gl_util](https://github.com/wlfrii/learn_OpenGL/tree/main/gl_util) - which can be found in my git repository and includes some rearranged interfaces for rendering on GLFW window.

Some other requirements can be found in the two required modules above.

## How to use

### I. Configure the library in your project 

#### I.1 For Linux / Mac OS (recommended)


### II. Use the library

A breif useage of this library is given as follows.

```c++
#include <lib_layer_renderer/lib_layer_renderer>

int main()
{
    // Set a rendering mode
    LayerRenderMode mode = LAYER_RENDER_LEFT;
    // Create a gl_util::Projection
    gl_util::Projection gl_proj(1120, 960, 540, 1920, 1080, 0.2, 150);
    // Create LayerRenderer object
    LayerRenderer renderer(gl_proj, mode);

    // (Optional)
    renderer.setView(...);
    renderer.setModel(...);

    // Add a layer to the renderer
    renderer.addLayers(std::make_shared<LayerCylinder>(
                       20, 4, glm::vec3(1.0, 0.0, 1.0)));

    // Show the rendered results
    renderer.render();

    return 0;
}
```

More __useage details of this library__ could be found in [`test/`](https://github.com/wlfrii/lib_layer_renderer/tree/main/test) folder.

Note, there should be a `models` folder that stores some gripper STL model. However, these files will not be uploaded due to usage restriction.