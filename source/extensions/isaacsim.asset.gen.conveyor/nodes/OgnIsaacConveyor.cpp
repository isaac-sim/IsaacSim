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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <OgnIsaacConveyorDatabase.h>

///
#include <omni/usd/UsdContext.h>
///

#include <carb/eventdispatcher/IEventDispatcher.h>
#include <carb/events/IEvents.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <omni/fabric/FabricUSD.h>
#include <omni/timeline/TimelineTypes.h>
#include <physxSchema/physxSurfaceVelocityAPI.h>
#include <pxr/pxr.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#if defined(__GNUC__) && !defined(__clang__)
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <usdrt/gf/vec.h>
#include <usdrt/scenegraph/usd/sdf/types.h>
#include <usdrt/scenegraph/usd/usd/attribute.h>
#include <usdrt/scenegraph/usd/usd/prim.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#if defined(__GNUC__) && !defined(__clang__)
#    pragma GCC diagnostic pop
#endif

#include <unordered_map>

namespace isaacsim
{
namespace asset
{
namespace gen
{
namespace conveyor
{

namespace
{

/**
 * @brief Token name of the MDL OmniPBR shader input that drives UV translation.
 */
const pxr::TfToken kTextureTranslateToken("inputs:texture_translate");

/**
 * @brief Returns true when Fabric Scene Delegate is enabled in the running session.
 *
 * The texture animation path must mirror writes into Fabric in this case so Hydra
 * picks up the per-frame UV update; otherwise the visible texture stays static
 * because USD authoring on the root layer is not synced back to Fabric for material
 * shader inputs in time for rendering.
 */
inline bool isFabricSceneDelegateEnabled()
{
    auto settings = carb::getCachedInterface<carb::settings::ISettings>();
    return settings && settings->getAsBool("/app/useFabricSceneDelegate");
}

} // namespace

/**
 * @brief Class that implements the conveyor belt node functionality
 */
class OgnIsaacConveyor
{
public:
    /**
     * @brief Initializes a new instance of the conveyor node
     * @param nodeObj Node object reference
     * @param instanceId Graph instance identifier
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacConveyorDatabase::sPerInstanceState<OgnIsaacConveyor>(nodeObj, instanceId);
        auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();

        // Stop-play handler restores the texture translations directly from the event
        // callback. We cannot rely on `requestCompute` to land before render: with
        // scheduling = compute-on-request and the OG action graph evaluator paused once
        // the timeline stops, the requested compute may be deferred or skipped entirely.
        // Doing the restore work synchronously here mirrors `BaseResetNode`'s approach
        // (see isaacsim/core/includes/BaseResetNode.h) and matches the pre-Kit-107.3
        // contract where the same callback dispatched the work.
        state.m_eventSubscription[0] = ed->observeEvent(
            carb::RStringKey("isaacsim.asset.gen.conveyor/OgnIsaacConveyor/StopPlay"),
            carb::eventdispatcher::kDefaultOrder, omni::timeline::kGlobalEventStop,
            [nodeObj, instanceId](const carb::eventdispatcher::Event& e)
            {
                if (!nodeObj.iNode->isValid(nodeObj))
                {
                    return;
                }
                auto& s = OgnIsaacConveyorDatabase::sPerInstanceState<OgnIsaacConveyor>(nodeObj, instanceId);
                // Order matters: flip m_isPlaying off first so any in-flight tick that
                // re-enters compute() after this callback returns sees the stopped state
                // and skips its texture-animation step. Otherwise that "tail tick" would
                // author one dt of UV translation on top of the value we just restored,
                // leaving a visible per-stop drift of exactly one delta.
                s.m_isPlaying = false;
                s.m_onStart = false;
                _restoreInitialTextureTranslations(s);
            });
        // Start-play handler flags m_onStart so compute() collects shader attributes for
        // texture animation on the first tick of the new sim run.
        auto usdContext = omni::usd::UsdContext::getContext();
        state.m_eventSubscription[1] = ed->observeEvent(
            carb::RStringKey("isaacsim.asset.gen.conveyor/OgnIsaacConveyor/StartPlay"),
            carb::eventdispatcher::kDefaultOrder,
            usdContext->stageEventName(omni::usd::StageEventType::eAnimationStartPlay),
            [nodeObj, instanceId](const carb::eventdispatcher::Event& e)
            {
                if (nodeObj.iNode->isValid(nodeObj))
                {
                    auto& s = OgnIsaacConveyorDatabase::sPerInstanceState<OgnIsaacConveyor>(nodeObj, instanceId);
                    s.m_isPlaying = true;
                    s.m_onStart = true;
                    // Drop any cached USDRT stage handle: a new sim run may rebuild the
                    // Fabric stage so we re-attach lazily on next compute(). Reset the
                    // attach-warning latch so a fresh failure on the new run is visible.
                    s.m_usdrtStage = nullptr;
                    s.m_stageId = 0;
                    s.m_loggedFabricMissing = false;
                    nodeObj.iNode->requestCompute(nodeObj);
                }
            });
    }

    /**
     * @brief Computes the conveyor belt physics and animation state
     * @param db Database containing node state and parameters
     * @return True if computation succeeded, false otherwise
     */
    static bool compute(OgnIsaacConveyorDatabase& db)
    {
        if (!db.inputs.enabled())
        {
            return true;
        }

        pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
        const auto& primPath = db.inputs.conveyorPrim();
        if (primPath.empty())
        {
            db.logError("No prim path found for the conveyor");
            return false;
        }

        const pxr::SdfPath conveyorSdfPath = omni::fabric::toSdfPath(primPath[0]);
        UsdPrim conveyorPrim = stage->GetPrimAtPath(conveyorSdfPath);

        auto& state = db.perInstanceState<OgnIsaacConveyor>();
        pxr::UsdPhysicsRigidBodyAPI physicsConveyor(conveyorPrim);
        pxr::GfVec3f currentVelocity;
        auto targetVelocity = db.inputs.direction() * db.inputs.velocity();

        if (!physicsConveyor)
        {
            db.logError("Selected Prim is not a Rigid Body");
            return false;
        }

        auto surfaceVelocity = pxr::PhysxSchemaPhysxSurfaceVelocityAPI::Apply(conveyorPrim);
        if (db.inputs.curved())
        {
            surfaceVelocity.GetSurfaceAngularVelocityAttr().Get(&currentVelocity);
        }
        else
        {
            surfaceVelocity.GetSurfaceVelocityAttr().Get(&currentVelocity);
        }

        // Lazy-attach the USDRT stage so we can mirror writes into Fabric when FSD is on.
        // This matches the OgnIsaacXPrimRadiusVisualizer pattern: getStageId from the OG context,
        // then attach via IStageReaderWriter so we share the same in-progress Fabric stage Hydra reads from.
        // If the stage reader-writer is unavailable on this tick, m_usdrtStage stays null and the
        // node falls back to USD-only authoring (correct under non-FSD Hydra). We log a single
        // warning per node-instance so this degraded path is observable in logs.
        if (state.m_usdrtStage == nullptr)
        {
            const GraphContextObj& context = db.abi_context();
            state.m_stageId = context.iContext->getStageId(context);
            auto iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
            if (iStageReaderWriter)
            {
                omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(state.m_stageId);
                state.m_usdrtStage = usdrt::UsdStage::Attach(state.m_stageId, stageInProgress);
            }
            if (state.m_usdrtStage == nullptr && !state.m_loggedFabricMissing)
            {
                state.m_loggedFabricMissing = true;
                CARB_LOG_WARN(
                    "OgnIsaacConveyor: USDRT stage attach failed (IStageReaderWriter unavailable). "
                    "Texture animation will fall back to USD-only authoring; UV updates may not "
                    "appear when Fabric Scene Delegate is enabled.");
            }
        }

        bool hasVelocityChanged = (currentVelocity - targetVelocity).GetLengthSq() > 1e-6f;
        if (state.m_onStart || hasVelocityChanged)
        {
            state.m_velocity = db.inputs.velocity();

            // Cycle the enabled attr to hardwire it to work on first sim
            surfaceVelocity.GetSurfaceVelocityEnabledAttr().Set(false);
            surfaceVelocity.GetSurfaceVelocityEnabledAttr().Set(true);

            if (db.inputs.curved())
            {
                surfaceVelocity.GetSurfaceAngularVelocityAttr().Set(targetVelocity);
            }
            else
            {
                surfaceVelocity.GetSurfaceVelocityAttr().Set(targetVelocity);
            }

            if (state.m_onStart)
            {
                // _collectShaderAttributes preserves any prior `initialValue` keyed by
                // shader path so we keep the true baseline across Pause/Play cycles. Only
                // flip m_onStart to false after the helper returns so a future early-exit
                // path does not leave us with no cached attributes and the flag cleared.
                _collectShaderAttributes(stage, state, conveyorPrim);
                state.m_onStart = false;
            }
        }

        // m_isPlaying guards against the "tail tick" that OnPlaybackTick can fire after
        // StopPlay has already restored the textures. Without it the post-stop tick
        // re-enters compute() and authors one full dt step on top of the restored
        // initial value, leaving the UV translation off-by-one-delta from rest.
        if (state.m_isPlaying && db.inputs.animateTexture() && state.m_velocity != 0 && db.inputs.onStep())
        {
            const pxr::GfVec2f deltaTranslation = pxr::GfVec2f(db.inputs.delta() * db.inputs.animateDirection()) *
                                                  state.m_velocity * db.inputs.animateScale();

            std::vector<pxr::GfVec2f> updatedTranslations;
            updatedTranslations.reserve(state.m_shaderAttributes.size());

            {
                pxr::UsdEditContext context(stage, stage->GetRootLayer());
                pxr::SdfChangeBlock changeBlock;
                for (auto& binding : state.m_shaderAttributes)
                {
                    pxr::GfVec2f textureTranslation;
                    binding.usdAttr.Get(&textureTranslation);
                    textureTranslation += deltaTranslation;
                    binding.usdAttr.Set(textureTranslation);
                    updatedTranslations.push_back(textureTranslation);
                }
            }

            // Mirror the per-frame UV translation into Fabric so Hydra/Fabric Scene Delegate
            // sees the change immediately. Without this, USD authoring on the root layer for
            // material shader inputs is not reliably reflected in the rendered frame when FSD
            // is enabled (see Kit FSD docs: enableFastDiffing default + material merging path).
            _writeFabricMirror(state, updatedTranslations);
        }

        return true;
    }

private:
    /**
     * @brief Per-shader UV-translation binding.
     * @details
     * USD and USDRT handles are stored together so they cannot drift out of lockstep across
     * partial-failure paths in `_collectShaderAttributes`. `usdrtAttr` is allowed to be null
     * (no USDRT stage attached, or USDRT attribute creation failed); the mirror helper checks
     * each handle individually.
     *
     * The per-binding `initialValue` is filled from `m_initialValuesByPath` so the baseline
     * survives Pause/Play cycles where the bindings vector is rebuilt. The map is the single
     * source of truth for "what value should Stop restore to".
     */
    struct ShaderAttributeBinding
    {
        pxr::UsdAttribute usdAttr;
        usdrt::UsdAttribute usdrtAttr{ nullptr };
        pxr::GfVec2f initialValue{ 0.0f, 0.0f };
        std::string shaderPath;
    };

    /**
     * @brief Collects every USD shader attribute that drives the conveyor belt's UV translation.
     * @details
     * Walks the conveyor's mesh prims, resolves the bound material via UsdShadeMaterialBindingAPI,
     * and for every shader prim under the material ensures `inputs:texture_translate` exists. The
     * USD attribute is recorded for authoring writes; if a USDRT stage is attached, the matching
     * USDRT attribute is also created/recorded so per-frame writes can be mirrored into Fabric.
     * Both handles are appended to the same `ShaderAttributeBinding` so they cannot desynchronise.
     *
     * Initial-value preservation: this helper is called on every Play (including resume from
     * Pause). For paths that already have a binding from a prior call we keep the cached
     * `initialValue` instead of resampling the (already-advanced) live USD value. Without
     * this, a Play/Pause/Play/Stop sequence would re-baseline at the value reached at the
     * second Play, leaving every Stop restore off by the in-between drift.
     *
     * Note: `inputs:texture_translate` is an OmniPBR-flavoured input; shaders that do not consume
     * it will still receive an authored attribute but it will be inert. This matches the original
     * behaviour and is the simplest portable contract.
     */
    static void _collectShaderAttributes(const pxr::UsdStagePtr& stage,
                                         OgnIsaacConveyor& state,
                                         const pxr::UsdPrim& conveyorPrim)
    {
        state.m_shaderAttributes.clear();

        // One edit-target scope for the whole walk; CreateAttribute and Set both need to land
        // on the root layer regardless of the active edit target the OG node was invoked under.
        pxr::UsdEditContext context(stage, stage->GetRootLayer());

        for (auto meshPrim : pxr::UsdPrimRange(conveyorPrim))
        {
            if (!pxr::UsdGeomImageable(meshPrim))
            {
                continue;
            }

            auto material = pxr::UsdShadeMaterialBindingAPI(meshPrim).ComputeBoundMaterial();
            for (auto shader : pxr::UsdPrimRange(material.GetPrim()))
            {
                const pxr::SdfPath shaderPath = shader.GetPrim().GetPath();
                const std::string shaderPathStr = shaderPath.GetString();
                auto shaderPrim = stage->OverridePrim(shaderPath);
                auto textureAttribute = shaderPrim.GetAttribute(kTextureTranslateToken);
                if (!textureAttribute)
                {
                    textureAttribute =
                        shaderPrim.CreateAttribute(kTextureTranslateToken, pxr::SdfValueTypeNames->Float2, false);
                    if (textureAttribute)
                    {
                        textureAttribute.Set(pxr::GfVec2f(0.0f, 0.0f));
                    }
                }

                if (!textureAttribute)
                {
                    continue;
                }

                // Single source of truth for the baseline: m_initialValuesByPath. Capture
                // the live USD value the first time we see this shader path and never
                // overwrite it. This survives Pause/Play cycles where the bindings vector
                // is rebuilt and OG-instance restarts that may rebuild the cache entirely.
                auto baselineIt = state.m_initialValuesByPath.find(shaderPathStr);
                if (baselineIt == state.m_initialValuesByPath.end())
                {
                    pxr::GfVec2f freshBaseline;
                    textureAttribute.Get(&freshBaseline);
                    baselineIt = state.m_initialValuesByPath.emplace(shaderPathStr, freshBaseline).first;
                }

                ShaderAttributeBinding binding;
                binding.usdAttr = textureAttribute;
                binding.shaderPath = shaderPathStr;
                binding.initialValue = baselineIt->second;

                // Cache a USDRT handle to the same shader attribute. Creating it on the USDRT
                // side ensures the attribute is present in Fabric even if USD-to-Fabric population
                // skipped it (e.g. populateAllAuthoredAttributes=false). The USD authoring write
                // above is preserved so the value persists if the stage is saved.
                if (state.m_usdrtStage)
                {
                    auto usdrtPrim = state.m_usdrtStage->GetPrimAtPath(usdrt::SdfPath(shaderPathStr));
                    if (usdrtPrim)
                    {
                        binding.usdrtAttr = usdrtPrim.GetAttribute(usdrt::TfToken(kTextureTranslateToken.GetString()));
                        if (!binding.usdrtAttr)
                        {
                            binding.usdrtAttr =
                                usdrtPrim.CreateAttribute(usdrt::TfToken(kTextureTranslateToken.GetString()),
                                                          usdrt::SdfValueTypeNames->Float2, false);
                            if (binding.usdrtAttr)
                            {
                                pxr::GfVec2f currentLiveValue;
                                textureAttribute.Get(&currentLiveValue);
                                binding.usdrtAttr.Set(usdrt::GfVec2f(currentLiveValue[0], currentLiveValue[1]));
                            }
                        }
                    }
                }

                state.m_shaderAttributes.push_back(std::move(binding));
            }
        }
    }

    /**
     * @brief Mirrors a list of texture translations into the cached USDRT shader attributes.
     * @details
     * Used to keep the rendered (Fabric) view in sync with the authored (USD) view. No-op when
     * Fabric Scene Delegate is disabled in the running session, since OmniHydra reads USD directly
     * and the USD-side `Set` is sufficient.
     */
    static void _writeFabricMirror(OgnIsaacConveyor& state, const std::vector<pxr::GfVec2f>& translations)
    {
        if (!state.m_usdrtStage || !isFabricSceneDelegateEnabled())
        {
            return;
        }

        const size_t count = std::min(translations.size(), state.m_shaderAttributes.size());
        for (size_t i = 0; i < count; ++i)
        {
            auto& usdrtAttr = state.m_shaderAttributes[i].usdrtAttr;
            if (usdrtAttr)
            {
                usdrtAttr.Set(usdrt::GfVec2f(translations[i][0], translations[i][1]));
            }
        }
    }

    /**
     * @brief Restores `inputs:texture_translate` to its captured initial value on every
     *        cached shader binding, in both USD and Fabric.
     * @details
     * Invoked synchronously from the StopPlay event handler. Authoring USD requires an
     * EditContext on the root layer because the OG node may have been the active edit
     * target during play; we wrap the writes in a SdfChangeBlock to coalesce notifications
     * for downstream consumers (Hydra, USDRT-population). The Fabric mirror is the path
     * that actually drives the rendered frame under FSD, so we always issue both writes.
     *
     * No-op when no shader attributes have been collected yet (e.g. stop fired before
     * the first compute() of a play session ran).
     */
    static void _restoreInitialTextureTranslations(OgnIsaacConveyor& state)
    {
        if (state.m_shaderAttributes.empty())
        {
            return;
        }

        std::vector<pxr::GfVec2f> initialStates;
        initialStates.reserve(state.m_shaderAttributes.size());

        pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
        if (stage)
        {
            pxr::UsdEditContext context(stage, stage->GetRootLayer());
            pxr::SdfChangeBlock changeBlock;
            for (auto& binding : state.m_shaderAttributes)
            {
                // The map is the authoritative baseline; binding.initialValue is just a
                // cached copy. Read from the map so a stale per-binding cache cannot
                // poison the restore.
                auto baselineIt = state.m_initialValuesByPath.find(binding.shaderPath);
                const pxr::GfVec2f baseline =
                    (baselineIt != state.m_initialValuesByPath.end()) ? baselineIt->second : binding.initialValue;
                if (binding.usdAttr)
                {
                    binding.usdAttr.Set(baseline);
                }
                initialStates.push_back(baseline);
            }
        }
        else
        {
            for (auto& binding : state.m_shaderAttributes)
            {
                auto baselineIt = state.m_initialValuesByPath.find(binding.shaderPath);
                const pxr::GfVec2f baseline =
                    (baselineIt != state.m_initialValuesByPath.end()) ? baselineIt->second : binding.initialValue;
                initialStates.push_back(baseline);
            }
        }

        _writeFabricMirror(state, initialStates);
    }

    std::vector<ShaderAttributeBinding> m_shaderAttributes;
    // Path -> first-ever-observed `inputs:texture_translate` value. Owns the baseline
    // contract: entries are only written when missing, never updated, so the bindings
    // vector can be rebuilt freely without losing the restore target.
    std::unordered_map<std::string, pxr::GfVec2f> m_initialValuesByPath;
    usdrt::UsdStageRefPtr m_usdrtStage{ nullptr };
    long m_stageId{ 0 };
    float m_velocity{ 0.0f };
    bool m_onStart{ false };
    bool m_isPlaying{ false };
    bool m_loggedFabricMissing{ false };
    carb::eventdispatcher::ObserverGuard m_eventSubscription[2];
};

REGISTER_OGN_NODE()

} // namespace conveyor
} // namespace gen
} // namespace asset
} // namespace isaacsim
