// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaacsim/foundation/utils/Backend.hpp"
#include "isaacsim/foundation/utils/Prim.hpp"
#include "isaacsim/foundation/utils/Semantics.hpp"
#include "isaacsim/foundation/utils/Stage.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <utility>

namespace nb = nanobind;
namespace objects = isaacsim::foundation::objects;
using namespace isaacsim::foundation::utils;

namespace
{

// Heap-allocated wrapper so the RAII guard lifetime is controlled explicitly via close().
class BackendGuardWrapper
{
public:
    BackendGuardWrapper(const std::string& backend, bool raiseOnUnsupported, bool raiseOnFallback)
        : m_guard(std::make_unique<BackendGuard>(backend, raiseOnUnsupported, raiseOnFallback))
    {
    }

    void close()
    {
        m_guard.reset(); // Reset on null unique_ptr is a no-op.
    }

private:
    std::unique_ptr<BackendGuard> m_guard;
};

// Heap-allocated wrapper so the RAII guard lifetime is controlled explicitly via close().
class StageGuardWrapper
{
public:
    explicit StageGuardWrapper(objects::Stage stage) : m_guard(std::make_unique<StageGuard>(std::move(stage)))
    {
    }

    void close()
    {
        m_guard.reset(); // Reset on null unique_ptr is a no-op.
    }

private:
    std::unique_ptr<StageGuard> m_guard;
};

} // namespace

NB_MODULE(_bindings, m)
{
    m.doc() = R"doc(Provide backend selection, active-stage access, prim traversal, and semantic-label utilities.

Calls execute synchronously and retain the Python GIL. Backend and active-stage overrides are local to the calling
thread; the default stage is process-wide.
)doc";

    // Backend.hpp
    nb::class_<BackendGuardWrapper>(m, "_BackendGuard",
                                    R"doc(Internal guard for a thread-local backend selection.

Use :func:`isaacsim.foundation.utils.backend.use_backend` instead of constructing this class directly.
Create and close a guard on the same thread.

Args:
    backend: Backend name to activate.
    raise_on_unsupported: Whether backend selection should fail when the active backend is unsupported.
    raise_on_fallback: Whether operations should fail when they use a fallback backend.
)doc")
        .def(nb::init<std::string, bool, bool>(), nb::arg("backend"), nb::arg("raise_on_unsupported") = false,
             nb::arg("raise_on_fallback") = false, "Initialize the backend guard for the calling thread.")
        .def("close", &BackendGuardWrapper::close,
             "Restore the backend context captured by this guard; repeated calls do nothing.");

    m.def("is_backend_set", &isBackendSet,
          R"doc(Check whether a backend is active on the calling thread.

Returns:
    Whether a thread-local backend is active.
)doc");
    m.def("should_raise_on_unsupported", &shouldRaiseOnUnsupported,
          R"doc(Check whether the active backend context rejects unsupported backends.

Returns:
    Whether unsupported backends should raise an exception.
)doc");
    m.def("should_raise_on_fallback", &shouldRaiseOnFallback,
          R"doc(Check whether the active backend context rejects fallback backends.

Returns:
    Whether use of a fallback backend should raise an exception.
)doc");
    m.def(
        "get_current_backend",
        [](const std::vector<std::string>& supported, std::optional<bool> raiseOnUnsupported)
        { return getCurrentBackend(supported, raiseOnUnsupported); },
        nb::arg("supported_backends"), nb::arg("raise_on_unsupported") = nb::none(),
        R"doc(Get the active backend supported by a caller.

Use the first supported backend when no backend is active. If the active backend is unsupported, either use the
first entry as a fallback or raise according to ``raise_on_unsupported`` and the active backend context.

Args:
    supported_backends: Ordered backend names accepted by the caller.
    raise_on_unsupported: Per-call override for rejecting an unsupported active backend, or ``None`` to use the
        active context setting.

Returns:
    Active backend name, or the first supported backend when using the default or a fallback.

Raises:
    ValueError: If ``supported_backends`` is empty.
    RuntimeError: If the active backend is unsupported and rejection is enabled.
)doc");
    // Stage.hpp
    nb::class_<StageGuardWrapper>(m, "_StageGuard",
                                  R"doc(Internal guard for a thread-local active stage.

Use :func:`isaacsim.foundation.utils.stage.use_stage` instead of constructing this class directly.
Create and close a guard on the same thread.

Args:
    stage: Stage to activate on the calling thread.
)doc")
        .def(nb::init<objects::Stage>(), nb::arg("stage"), "Initialize the stage guard for the calling thread.")
        .def("close", &StageGuardWrapper::close,
             "Restore the active stage captured by this guard; repeated calls do nothing.");

    m.def("set_default_stage", &setDefaultStage, nb::arg("stage"),
          nb::sig("def set_default_stage(stage: isaacsim.foundation.objects.Stage) -> None"),
          R"doc(Set the process-wide default stage.

Args:
    stage: Valid stage to use when the calling thread has no active-stage override.

Raises:
    RuntimeError: If ``stage`` is invalid.
)doc");
    m.def("get_default_stage", &getDefaultStage, nb::sig("def get_default_stage() -> isaacsim.foundation.objects.Stage"),
          R"doc(Get the process-wide default stage.

Returns:
    Current default stage.

Raises:
    RuntimeError: If no default stage has been set.
)doc");
    m.def("get_active_stage", &getActiveStage, nb::sig("def get_active_stage() -> isaacsim.foundation.objects.Stage"),
          R"doc(Get the active stage for the calling thread.

Return the thread-local override when it is valid, or the process-wide default stage otherwise.

Returns:
    Active stage.

Raises:
    RuntimeError: If neither a valid thread-local stage nor a default stage exists.
)doc");
    // Prim.hpp
    m.def("find_matching_prim_paths", &findMatchingPrimPaths, nb::arg("path"), nb::kw_only(), nb::arg("traverse") = false,
          R"doc(Find prim paths on the active stage that match a path expression.

For a literal path, ``traverse`` includes descendants. For a pattern, ``traverse`` chooses between matching each
path segment level by level and matching the complete expression against every prim path.

Args:
    path: Literal USD prim path or regular-expression pattern.
    traverse: Whether to traverse descendants for a literal path or match a pattern against the full stage.

Returns:
    Matching prim paths in stage traversal order.

Raises:
    RuntimeError: If no active or default stage is available.
)doc");
    m.def("get_all_matching_child_prims", &getAllMatchingChildPrims, nb::arg("path"), nb::kw_only(),
          nb::arg("predicate"), nb::arg("include_self") = false, nb::arg("max_depth") = nb::none(),
          R"doc(Get all matching prims in a subtree using breadth-first traversal.

The predicate is called synchronously with each visited prim path. Depth is measured from ``path`` even when the
root is excluded.

Args:
    path: USD path of the subtree root on the active stage.
    predicate: Callable that returns whether to include a visited prim path.
    include_self: Whether to test the subtree root.
    max_depth: Maximum depth to visit relative to the root, or ``None`` for unlimited traversal.

Returns:
    Matching prim paths in breadth-first order.

Raises:
    RuntimeError: If no active or default stage is available.
)doc");
    m.def("get_first_matching_child_prim", &getFirstMatchingChildPrim, nb::arg("path"), nb::kw_only(),
          nb::arg("predicate"), nb::arg("include_self") = false, nb::arg("max_depth") = nb::none(),
          R"doc(Get the first matching prim in a subtree using breadth-first traversal.

The predicate is called synchronously with each visited prim path. Depth is measured from ``path`` even when the
root is excluded.

Args:
    path: USD path of the subtree root on the active stage.
    predicate: Callable that returns whether to select a visited prim path.
    include_self: Whether to test the subtree root first.
    max_depth: Maximum depth to visit relative to the root, or ``None`` for unlimited traversal.

Returns:
    First matching prim path, or ``None`` if no prim matches.

Raises:
    RuntimeError: If no active or default stage is available.
)doc");
    m.def("get_first_matching_parent_prim", &getFirstMatchingParentPrim, nb::arg("path"), nb::kw_only(),
          nb::arg("predicate"), nb::arg("include_self") = false,
          R"doc(Get the first matching ancestor of a prim.

Walk toward the stage pseudoroot and call the predicate synchronously for each visited path. The pseudoroot is not
tested or returned.

Args:
    path: USD prim path whose ancestors to search on the active stage.
    predicate: Callable that returns whether to select a visited prim path.
    include_self: Whether to test ``path`` before its ancestors.

Returns:
    First matching prim path, or ``None`` if no ancestor matches.

Raises:
    RuntimeError: If no active or default stage is available.
)doc");
    // Semantics.hpp
    m.def("add_labels", &addLabels, nb::arg("path"), nb::kw_only(), nb::arg("labels"), nb::arg("taxonomy") = "class",
          R"doc(Add unique semantic labels to a prim under a taxonomy.

Apply the semantic-label schema when needed and preserve existing labels. Descendants are not modified.

Args:
    path: USD path of the target prim on the active stage.
    labels: Label or labels to add.
    taxonomy: Taxonomy under which to store the labels.

Raises:
    RuntimeError: If no active or default stage is available.
)doc");
    m.def("get_labels", &getLabels, nb::arg("path"), nb::kw_only(), nb::arg("include_descendants") = false,
          R"doc(Get semantic labels grouped by taxonomy.

When descendants are included, concatenate their labels in traversal order; labels repeated across prims remain in
the result.

Args:
    path: USD path of the target prim on the active stage.
    include_descendants: Whether to aggregate labels from the complete subtree.

Returns:
    Taxonomy names mapped to their ordered label lists.

Raises:
    RuntimeError: If no active or default stage is available.
)doc");
    m.def("remove_labels", &removeLabels, nb::arg("path"), nb::kw_only(), nb::arg("labels"),
          nb::arg("taxonomy") = nb::none(), nb::arg("include_descendants") = false,
          R"doc(Remove selected semantic labels from a prim or subtree.

Keep semantic taxonomy schemas applied even when their label lists become empty.

Args:
    path: USD path of the target prim on the active stage.
    labels: Label or labels to remove.
    taxonomy: Taxonomy to modify, or ``None`` to modify every taxonomy.
    include_descendants: Whether to apply the removal to the complete subtree.

Raises:
    RuntimeError: If no active or default stage is available.
)doc");
    m.def("remove_all_labels", &removeAllLabels, nb::arg("path"), nb::kw_only(), nb::arg("remove_taxonomies") = false,
          nb::arg("include_descendants") = false,
          R"doc(Remove all semantic labels from a prim or subtree.

Args:
    path: USD path of the target prim on the active stage.
    remove_taxonomies: Whether to remove semantic taxonomy schemas as well as their labels.
    include_descendants: Whether to apply the removal to the complete subtree.

Raises:
    RuntimeError: If no active or default stage is available.
)doc");
}
