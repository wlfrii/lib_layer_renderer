/**--------------------------------------------------------------------
 *																		
 *   				    Layer renderer library
 *																		
 * Description:													
 * This file is header file of lib_layer_renderer. You can redistribute it
 *  and or modify it to construct your own project. It is wellcome to use 
 * this library in your scientific research work.
 * 
 * @file 		lib_layer_renderer.h 
 * 
 * @brief 		The header file for the library
 * 
 * @author		Longfei Wang
 * 
 * @date		2022/04/01
 * 
 * @license		
 * 
 * Copyright (C) 2021-Now Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_LAYER_RENDERER_LIB_LF
#define LIB_LAYER_RENDERER_LIB_LF

// Renderer
#include "lib_layer_renderer/layer_renderer.h"

// Layers
#include "lib_layer_renderer/layer_background.h"
#include "lib_layer_renderer/layer_circle.h"
#include "lib_layer_renderer/layer_cone.h"
#include "lib_layer_renderer/layer_coordinate.h"
#include "lib_layer_renderer/layer_cylinder.h"
#include "lib_layer_renderer/layer_gripper.h"
#include "lib_layer_renderer/layer_segment.h"
#include "lib_layer_renderer/layer_util.h"


// Some explicit template class
namespace mmath {

using Linef = Line<float>;
using GaussianCurvef = GaussianCurve<float>;

}

#endif // LIB_LAYER_RENDERER_LIB_LF
