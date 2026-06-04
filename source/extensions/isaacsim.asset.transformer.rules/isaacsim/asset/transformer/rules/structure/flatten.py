# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Flatten rule for USD asset transformation."""

from __future__ import annotations

import os

from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Sdf, Usd

# Default configuration values
_DEFAULT_OUTPUT_PATH: str = "base.usda"
_DEFAULT_CLEAR_VARIANTS: bool = True
_DEFAULT_SELECTED_VARIANTS: dict[str, str] = {}
_DEFAULT_CASE_INSENSITIVE: bool = True


class FlattenRule(RuleInterface):
    """Flatten the original input stage into a single layer.

    Opens the original input asset (via input_stage_path from args), clears
    variant selections before flattening to ensure a neutral base representation,
    and exports the flattened result. Operating on the original preserves
    relative asset paths that would be broken after initial processing.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters for output path, variant clearing, and variant selection.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return [
            RuleConfigurationParam(
                name="output_path",
                display_name="Output Path",
                param_type=str,
                description="Relative path within package root to export the flattened layer",
                default_value=_DEFAULT_OUTPUT_PATH,
            ),
            RuleConfigurationParam(
                name="clear_variants",
                display_name="Clear Variant Selections",
                param_type=bool,
                description="Clear all variant selections before flattening to produce a neutral base",
                default_value=_DEFAULT_CLEAR_VARIANTS,
            ),
            RuleConfigurationParam(
                name="selected_variants",
                display_name="Selected Variants",
                param_type=dict,
                description="Dictionary mapping variant set names to variant selections. "
                "Only applies to variant sets that exist on the asset default prim.",
                default_value=_DEFAULT_SELECTED_VARIANTS,
            ),
            RuleConfigurationParam(
                name="case_insensitive",
                display_name="Case Insensitive",
                param_type=bool,
                description="If True, match variant names case-insensitively when applying selections.",
                default_value=_DEFAULT_CASE_INSENSITIVE,
            ),
        ]

    def process_rule(self) -> str | None:
        """Flatten the original input stage and export to the output path.

        Opens a private :class:`Usd.Stage` from ``args["input_stage_path"]``
        (intentionally ignoring any ``args["input_stage"]`` object the caller
        may have supplied), applies variant selections if configured, clears
        remaining variant selections if configured, flattens, and exports.
        This preserves relative paths that would be broken after initial
        processing.

        Variant edits are routed to that private stage's session layer via
        :class:`Usd.EditContext`. Two guarantees follow:

        1. The shared root ``Sdf.Layer`` (potentially shared with other
           Stages through USD's process-wide layer cache, notably the
           editor's active Stage) is never mutated. Mutating it fires
           change notifications that have been observed to invalidate
           Hydra render product prims mid-frame and crash ``librtx.hydra``.
        2. A caller-owned Stage passed via ``args["input_stage"]`` is never
           touched -- its session layer (which in the editor commonly
           carries user-driven overrides such as visibility toggles, purpose
           settings, and camera opinions) is left intact.

        ``Usd.Stage.Open`` is cheap because USD's layer cache reuses any
        already-loaded root layer; only the new Stage's session layer is
        created fresh.

        Returns:
            Path to the flattened stage for subsequent rules to use.

        Example:

        .. code-block:: python

            output_path = rule.process_rule()

        """
        params = self.args.get("params", {}) or {}

        output_path = params.get("output_path") or _DEFAULT_OUTPUT_PATH
        clear_variants = params.get("clear_variants", _DEFAULT_CLEAR_VARIANTS)
        selected_variants = params.get("selected_variants") or _DEFAULT_SELECTED_VARIANTS
        case_insensitive = params.get("case_insensitive", _DEFAULT_CASE_INSENSITIVE)

        # Get the original input stage path
        input_stage_path = self.args.get("input_stage_path")
        if not input_stage_path:
            self.log_operation("No input_stage_path provided, cannot flatten original source")
            return None

        self.log_operation(
            f"FlattenRule start input={input_stage_path} output={output_path} "
            f"clear_variants={clear_variants} selected_variants={selected_variants} "
            f"case_insensitive={case_insensitive}"
        )

        # Always open a fresh, private Stage from the input path; do not
        # use ``args["input_stage"]`` even if the caller provided one.
        # The fresh Stage owns an empty session layer we can author into
        # freely without disturbing any caller-held Stage's session layer.
        # The Stage is local to this function and is garbage-collected on
        # return, so no explicit cleanup of the session layer is required.
        input_stage = Usd.Stage.Open(input_stage_path)
        if not input_stage:
            self.log_operation(f"Failed to open input stage: {input_stage_path}")
            return None

        session_layer = input_stage.GetSessionLayer()
        if session_layer is None:
            # ``Usd.Stage.Open`` always provides a session layer; this
            # branch is purely defensive for future API changes.
            session_layer = Sdf.Layer.CreateAnonymous("flatten-session.usda")
            input_stage = Usd.Stage.Open(input_stage.GetRootLayer(), session_layer)

        with Usd.EditContext(input_stage, session_layer):
            if clear_variants:
                self._clear_all_variant_selections(input_stage)
            if selected_variants:
                self._apply_variant_selections(input_stage, selected_variants, case_insensitive)
            flattened_layer = input_stage.Flatten()

        if not flattened_layer:
            self.log_operation("Failed to flatten input stage")
            return None

        # Resolve output path
        output_abs_path = os.path.join(self.package_root, os.path.join(self.destination_path, output_path))
        os.makedirs(os.path.dirname(output_abs_path), exist_ok=True)

        # Check if the layer is already cached by USD (e.g., opened by manager.py)
        cached_layer = Sdf.Layer.Find(output_abs_path)
        if cached_layer:
            # Transfer content to the cached layer and save to avoid cache conflicts
            self.log_operation(f"Layer cached in USD registry, transferring content: {output_abs_path}")
            cached_layer.TransferContent(flattened_layer)
            if not cached_layer.Save():
                self.log_operation(f"Failed to save cached layer: {output_abs_path}")
                return None
        else:
            # No cached layer, export directly
            if os.path.exists(output_abs_path):
                os.remove(output_abs_path)
                self.log_operation(f"Removed existing file: {output_abs_path}")
            if not flattened_layer.Export(output_abs_path):
                self.log_operation(f"Failed to export flattened layer to: {output_abs_path}")
                return None

        self.log_operation(f"Exported flattened layer to: {output_abs_path}")
        self.add_affected_stage(output_path)
        self.log_operation("FlattenRule completed")

        # Return the path so manager can switch to the flattened stage
        return output_abs_path

    def _apply_variant_selections(
        self, stage: Usd.Stage, selected_variants: dict[str, str], case_insensitive: bool = True
    ) -> None:
        """Apply variant selections on the stage default prim.

        Sets variant selections for variant sets that exist on the default prim.
        Variant sets not present on the prim are skipped. If the requested variant
        does not exist within the variant set, the selection is kept as-is.

        Args:
            stage: The stage containing the default prim.
            selected_variants: Dictionary mapping variant set names to variant selections.
            case_insensitive: If True, match variant names case-insensitively.

        """
        default_prim = stage.GetDefaultPrim()
        if not default_prim or not default_prim.IsValid():
            self.log_operation("No valid default prim found, skipping variant selection")
            return

        variant_sets = default_prim.GetVariantSets()
        applied_count = 0

        for variant_set_name, variant_selection in selected_variants.items():
            if variant_sets.HasVariantSet(variant_set_name):
                variant_set = variant_sets.GetVariantSet(variant_set_name)
                available_variants = variant_set.GetVariantNames()

                # Find the matching variant, using case-insensitive matching if enabled
                matched_variant = None
                if variant_selection in available_variants:
                    matched_variant = variant_selection
                elif case_insensitive:
                    # Search for a case-insensitive match
                    variant_selection_lower = variant_selection.lower()
                    for available in available_variants:
                        if available.lower() == variant_selection_lower:
                            matched_variant = available
                            self.log_operation(f"Case-insensitive match: '{variant_selection}' -> '{matched_variant}'")
                            break

                if matched_variant:
                    variant_set.SetVariantSelection(matched_variant)
                    self.log_operation(f"Set variant '{variant_set_name}' to '{matched_variant}'")
                else:
                    self.log_operation(
                        f"Variant '{variant_selection}' not found in '{variant_set_name}', keeping "
                        f"existing selection '{variant_set.GetVariantSelection()}'"
                    )
                applied_count += 1
            else:
                self.log_operation(f"Variant set '{variant_set_name}' not found on default prim, skipping")

        if applied_count > 0:
            self.log_operation(f"Applied {applied_count} variant selection(s)")
        else:
            self.log_operation("No variant selections applied")

    def _clear_all_variant_selections(self, stage: Usd.Stage) -> None:
        """Block variant selections on all prims via the current edit target.

        Walks the composed stage and writes a *block* opinion for every
        authored variant selection through :class:`UsdVariantSet`. Block
        opinions override weaker selections so the flattened output picks
        no variant -- without mutating the root layer.

        Must be called inside a :class:`Usd.EditContext` whose target is the
        session layer (or another non-root layer); see
        :meth:`process_rule` for the rationale.

        Args:
            stage: The stage to clear variant selections from.

        """
        cleared_count = 0

        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if not prim.IsValid():
                continue

            variant_sets = prim.GetVariantSets()
            for vs_name in variant_sets.GetNames():
                if not variant_sets.HasVariantSet(vs_name):
                    continue
                variant_set = variant_sets.GetVariantSet(vs_name)
                if variant_set.GetVariantSelection():
                    variant_set.BlockVariantSelection()
                    cleared_count += 1

        if cleared_count > 0:
            self.log_operation(f"Cleared {cleared_count} variant selection(s)")
        else:
            self.log_operation("No variant selections to clear")
