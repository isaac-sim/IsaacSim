// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef USD_PCH_H
#define USD_PCH_H

// !!! DO NOT INCLUDE THIS FILE IN A HEADER !!!
// When you include this file in a cpp file, add the file name to premake5.lua's pchFiles list!

// The usd headers drag in heavy dependencies and are very slow to build.
// Make it PCH to speed up building time.

#ifdef _MSC_VER
#    pragma warning(push)
#    pragma warning(disable : 4244) // = Conversion from double to float / int to float
#    pragma warning(disable : 4267) // conversion from size_t to int
#    pragma warning(disable : 4305) // argument truncation from double to float
#    pragma warning(disable : 4800) // int to bool
#    pragma warning(disable : 4996) // call to std::copy with parameters that may be unsafe
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    define WIN32_LEAN_AND_MEAN
#    include <Windows.h> // Include this here so we can curate
#elif defined(__GNUC__)
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#    pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#    pragma GCC diagnostic ignored "-Wunused-function"
// This suppresses deprecated header warnings, which is impossible with pragmas.
// Alternative is to specify -Wno-deprecated build option, but that disables other useful warnings too.
#    ifdef __DEPRECATED
#        define OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#        undef __DEPRECATED
#    endif
#endif

// Include cstdio here so that vsnprintf is properly declared. This is necessary because pyerrors.h has
// #define vsnprintf _vsnprintf which later causes <cstdio> to declare std::_vsnprintf instead of the correct and proper
// std::vsnprintf. By doing it here before everything else, we avoid this nonsense.
#include <pxr/base/arch/stackTrace.h>
#include <pxr/base/arch/threads.h>
#include <pxr/base/gf/api.h>
#include <pxr/base/gf/camera.h>
#include <pxr/base/gf/frustum.h>
#include <pxr/base/gf/matrix3f.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/matrix4f.h>
#include <pxr/base/gf/quaternion.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/plug/notice.h>
#include <pxr/base/plug/plugin.h>
#include <pxr/base/tf/hashmap.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/trace/reporter.h>
#include <pxr/base/trace/trace.h>
#include <pxr/base/vt/value.h>
#include <pxr/base/work/loops.h>
#include <pxr/base/work/threadLimits.h>
#include <pxr/imaging/hd/basisCurves.h>
#include <pxr/imaging/hd/camera.h>
#include <pxr/imaging/hd/engine.h>
#include <pxr/imaging/hd/extComputation.h>
#include <pxr/imaging/hd/field.h>
#include <pxr/imaging/hd/flatNormals.h>
#include <pxr/imaging/hd/instancer.h>
#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/material.h>
#include <pxr/imaging/hd/mesh.h>
#include <pxr/imaging/hd/meshUtil.h>
#include <pxr/imaging/hd/points.h>
#include <pxr/imaging/hd/renderBuffer.h>
#include <pxr/imaging/hd/renderIndex.h>
#include <pxr/imaging/hd/renderPass.h>
#include <pxr/imaging/hd/renderPassState.h>
#include <pxr/imaging/hd/rendererPluginRegistry.h>
#include <pxr/imaging/hd/resourceRegistry.h>
#include <pxr/imaging/hd/rprim.h>
#include <pxr/imaging/hd/smoothNormals.h>
#include <pxr/imaging/hd/sprim.h>
#include <pxr/imaging/hd/vertexAdjacency.h>
#include <pxr/imaging/hdx/tokens.h>
#include <pxr/imaging/pxOsd/tokens.h>
#include <pxr/pxr.h>
#include <pxr/usd/ar/resolver.h>
#include <pxr/usd/ar/resolverContext.h>
#include <pxr/usd/ar/resolverContextBinder.h>
#include <pxr/usd/ar/resolverScopedCache.h>
#include <pxr/usd/kind/registry.h>
#include <pxr/usd/pcp/layerStack.h>
#include <pxr/usd/pcp/site.h>
#include <pxr/usd/sdf/attributeSpec.h>
#include <pxr/usd/sdf/changeList.h>
#include <pxr/usd/sdf/copyUtils.h>
#include <pxr/usd/sdf/fileFormat.h>
#include <pxr/usd/sdf/layerStateDelegate.h>
#include <pxr/usd/sdf/layerUtils.h>
#include <pxr/usd/sdf/relationshipSpec.h>
#include <pxr/usd/sdr/registry.h>
#include <pxr/usd/sdr/shaderNode.h>
#include <pxr/usd/sdr/shaderProperty.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/notice.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cone.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>
#include <pxr/usd/usdLux/cylinderLight.h>
#include <pxr/usd/usdLux/diskLight.h>
#include <pxr/usd/usdLux/distantLight.h>
#include <pxr/usd/usdLux/domeLight.h>

#include <cstdio>
// https://github.com/PixarAnimationStudios/USD/commit/7540fdf3b2aa6b6faa0fce8e7b4c72b756286f51
#if PXR_VERSION >= 2111
#    include <pxr/usd/usdLux/lightAPI.h>
#endif
#include <pxr/usd/usdLux/rectLight.h>
#include <pxr/usd/usdLux/sphereLight.h>
#include <pxr/usd/usdLux/tokens.h>
#include <pxr/usd/usdShade/tokens.h>
#include <pxr/usd/usdSkel/animation.h>
#include <pxr/usd/usdSkel/root.h>
#include <pxr/usd/usdSkel/skeleton.h>
#include <pxr/usd/usdSkel/tokens.h>
#include <pxr/usd/usdUtils/stageCache.h>
#include <pxr/usd/usdVol/tokens.h>
#include <pxr/usdImaging/usdImaging/delegate.h>


// -- Hydra

#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/imaging/hd/renderIndex.h>
#include <pxr/imaging/hd/rendererPlugin.h>
#include <pxr/imaging/hd/sceneDelegate.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/imaging/hdx/taskController.h>
#include <pxr/usdImaging/usdImaging/gprimAdapter.h>
#include <pxr/usdImaging/usdImaging/indexProxy.h>
#include <pxr/usdImaging/usdImaging/tokens.h>
#include <pxr/usdImaging/usdVolImaging/tokens.h>

// -- nv extensions
// #include <audioSchema/sound.h>

// -- omni.usd
// clang-format off
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UtilsIncludes.h>
// clang-format on

#ifdef _MSC_VER
#    pragma warning(pop)
#elif defined(__GNUC__)
#    pragma GCC diagnostic pop
#    ifdef OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#        define __DEPRECATED
#        undef OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#    endif
#endif

#endif // #ifndef USD_PCH_H
