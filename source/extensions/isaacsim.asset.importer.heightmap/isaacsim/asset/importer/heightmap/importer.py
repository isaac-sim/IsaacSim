# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Heightmap importer utilities."""

from __future__ import annotations

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.usd
from isaacsim.core.experimental.objects import GroundPlane
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

__all__ = [
    "HeightmapImporter",
    "cube_prototype_path",
    "generated_paths",
    "instances_path",
    "is_importer_generated",
]

# Heightmap generation constants
DEFAULT_CUBE_HEIGHT = 2.0
DEFAULT_CUBE_COLOR = (0.5, 0.5, 0.5)
OCCUPIED_PIXEL_THRESHOLD = 127  # Pixels darker than this value are considered occupied
GROUND_PLANE_MARGIN = 1.0
DEFAULT_LIGHT_INTENSITY = 2000

# Image processing constants
LARGE_IMAGE_THRESHOLD = 1000000  # Show progress message for images > 1MP

# USD Scene paths
WORLD_PATH = "/World"
GROUND_PLANE_PATH = "/World/groundPlane"
LIGHT_PATH = "/World/defaultLight"
OCCUPANCY_MAP_PATH = "/World/occupancyMap"

# How many ``occupancyMap``, ``occupancyMap_01``, ... candidates to try before giving up.
# A stage already carrying this many terrains is not one another will improve.
MAX_TERRAIN_PATH_CANDIDATES = 100

# Stamped on every prim the importer creates so that regenerating can tell its own previous
# output apart from content the user authored at the same path.
GENERATED_MARKER_ATTR = "isaacsim:heightmapImporter:generated"


def generated_paths(
    create_ground_plane: bool, create_lighting: bool, terrain_path: str = OCCUPANCY_MAP_PATH
) -> list[str]:
    """Return the prim paths a generation run with these options writes to.

    Writing to them is not the same as replacing what is there: a run that is not allowed
    to overwrite skips an occupied path rather than clearing it.

    Args:
        create_ground_plane: Whether the run creates a ground plane.
        create_lighting: Whether the run creates a light.
        terrain_path: Where the terrain itself goes.

    Returns:
        The paths the run writes to.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.heightmap.importer import generated_paths

        >>> generated_paths(create_ground_plane=True, create_lighting=False)
        ['/World/occupancyMap', '/World/groundPlane']
    """
    paths = [terrain_path]
    if create_ground_plane:
        paths.append(GROUND_PLANE_PATH)
    if create_lighting:
        paths.append(LIGHT_PATH)
    return paths


def instances_path(terrain_path: str) -> str:
    """Return the point instancer path beneath ``terrain_path``.

    Args:
        terrain_path: Root of the generated terrain.

    Returns:
        Path of the point instancer holding the occupied cells.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.heightmap.importer import instances_path

        >>> instances_path("/World/occupancyMap")
        '/World/occupancyMap/occupiedInstances'
    """
    return f"{terrain_path}/occupiedInstances"


def cube_prototype_path(terrain_path: str) -> str:
    """Return the instanced cube's path beneath ``terrain_path``.

    Args:
        terrain_path: Root of the generated terrain.

    Returns:
        Path of the cube prototype the instancer references.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.heightmap.importer import cube_prototype_path

        >>> cube_prototype_path("/World/occupancyMap")
        '/World/occupancyMap/occupiedInstances/occupiedCube'
    """
    return f"{instances_path(terrain_path)}/occupiedCube"


def _has_generated_marker(prim: Usd.Prim) -> bool:
    """Check a single prim for the importer's marker attribute.

    Args:
        prim: Prim to check.

    Returns:
        True if the prim carries the marker.
    """
    attribute = prim.GetAttribute(GENERATED_MARKER_ATTR)
    if not attribute or not attribute.Get():
        return False
    # A composed value is not evidence of authorship. The user can inherit the marker from
    # a class, or pick it up from a reference, and reading the attribute cannot tell that
    # apart from a prim this importer stamped. Ownership means we wrote it on this prim.
    own_path = prim.GetPath()
    return any(
        spec.path.GetPrimPath().StripAllVariantSelections() == own_path
        for spec in attribute.GetPropertyStack(Usd.TimeCode.Default())
    )


def _has_unselected_variants(prim: Usd.Prim) -> bool:
    """Check whether any variant set on ``prim`` has a variant that is not in effect.

    The selected variant's content is composed and visible, so a set offering nothing else
    hides nothing. It is the variants left unselected that could define prims at paths
    reading as empty, and no amount of inspecting the composed stage will reveal them.

    Args:
        prim: Prim whose variant sets to examine.

    Returns:
        True if some variant other than the current selection exists.
    """
    variant_sets = prim.GetVariantSets()
    return any(
        name != variant_sets.GetVariantSet(set_name).GetVariantSelection()
        for set_name in variant_sets.GetNames()
        for name in variant_sets.GetVariantSet(set_name).GetVariantNames()
    )


def _prim_spec_paths(stage: Usd.Stage) -> set[str]:
    """Collect every prim path any layer in ``stage``'s own layer stack defines.

    Asked of the layer stack rather than one layer, so a spec a sublayer contributes is
    found however deeply it is nested. Resolving the sublayer identifiers by hand gets
    this wrong for anonymous layers, which do not resolve relative to their parent.

    Variant selections are stripped, so a spec authored at
    ``/World{layout=withMap}occupancyMap`` is reported as ``/World/occupancyMap``. A
    lookup by composed path would miss it, and unmuting plus selecting that variant is
    all it takes to bring it back under whatever was written there.

    Args:
        stage: Stage whose layer stack to read.

    Returns:
        The prim paths those layers define.
    """
    paths: set[str] = set()
    for layer in stage.GetLayerStack(includeSessionLayers=False):
        layer.Traverse(Sdf.Path.absoluteRootPath, lambda p: paths.add(p.StripAllVariantSelections().pathString))
    return paths


def _has_unloaded_payload_above(stage: Usd.Stage, path: str) -> bool:
    """Check whether an ancestor of ``path`` on ``stage`` carries a payload left unloaded.

    What the payload defines is unknown without opening it, and it could well be this very
    path, so the path counts as taken.

    Args:
        stage: Stage to inspect.
        path: Prim path whose ancestors to examine.

    Returns:
        True if some ancestor has an authored payload that is not loaded.
    """
    ancestor = Sdf.Path(path).GetParentPath()
    while not ancestor.isEmpty and ancestor != Sdf.Path.absoluteRootPath:
        prim = stage.GetPrimAtPath(ancestor)
        if prim.IsValid() and prim.HasAuthoredPayloads() and not prim.IsLoaded():
            return True
        ancestor = ancestor.GetParentPath()
    return False


def _is_a_path_generation_may_write(path: str) -> bool:
    """Check whether a path is one some run of this importer could write to.

    Covers the fixed paths and the suffixed terrain paths a run that may not overwrite
    falls back to, since which of those a given run picks is not known until it looks.

    Args:
        path: Absolute prim path to test.

    Returns:
        True if some run could write to this path.
    """
    return path in (OCCUPANCY_MAP_PATH, GROUND_PLANE_PATH, LIGHT_PATH) or path.startswith(f"{OCCUPANCY_MAP_PATH}_")


def _a_variant_could_claim_a_generated_path(stage: Usd.Stage) -> bool:
    """Check whether a variant of ``stage`` could define a path generation writes to.

    A variant reaches one of those paths two ways. It can define the path outright, which
    appears as a variant-encoded spec at that path -- a spec at
    ``/World{layout=withMap}occupancyMap`` means some variant defines
    ``/World/occupancyMap``. Or it can carry a composition arc at or above the path, which
    contributes whatever the target holds. No scan of this stage's layers can enumerate
    that, since the target need not even be reachable yet, so an arc is taken at its word:
    the variant might contribute anything beneath it.

    Selected variants count too. One that composes is found by ordinary means anyway, and
    answering for every variant avoids asking a question whose answer changes with the
    selection.

    A variant that only overrides attributes -- a lookdev or time-of-day set that rebinds
    materials and defines no prims -- matches neither test, which is the point of asking.
    Treating every variant set as a hazard would rule out most authored stages.

    Args:
        stage: Stage whose layer stack to read.

    Returns:
        True if some variant could define a path generation writes to.
    """
    could_claim = False

    def inspect(path: Sdf.Path, layer: Sdf.Layer) -> None:
        nonlocal could_claim
        if could_claim or not path.ContainsPrimVariantSelection():
            return
        own_path = path.StripAllVariantSelections()
        if _is_a_path_generation_may_write(own_path.pathString):
            could_claim = True
            return
        if not any(
            Sdf.Path(generated).HasPrefix(own_path) for generated in (OCCUPANCY_MAP_PATH, GROUND_PLANE_PATH, LIGHT_PATH)
        ):
            return
        spec = layer.GetObjectAtPath(path)
        if spec is None:
            return
        # A variant selection path names a variant spec, and its arcs are on the prim spec
        # it wraps. Deeper paths are prim specs already.
        prim_spec = spec.primSpec if isinstance(spec, Sdf.VariantSpec) else spec
        if not isinstance(prim_spec, Sdf.PrimSpec):
            # The variant set itself, which holds the variants and carries no arcs of its own.
            return
        could_claim = any(
            getattr(prim_spec, field).GetAddedOrExplicitItems()
            for field in ("referenceList", "payloadList", "inheritPathList", "specializesList")
        )

    for layer in stage.GetLayerStack(includeSessionLayers=True):
        layer.Traverse(Sdf.Path.absoluteRootPath, lambda path, layer=layer: inspect(path, layer))
    return could_claim


def _hides_namespace_below(prim: Usd.Prim) -> bool:
    """Check whether a prim is currently stopping composition of its own namespace.

    Each of these makes paths beneath the prim resolve to nothing on the composed stage,
    so asking whether such a path is occupied cannot be answered by looking at it.

    Deliberately narrower than :func:`_conceals_descendants`: it asks only what is hidden
    *now*, not what a prim might be carrying. A payload that resolved and loaded composes
    its content into the namespace, so the paths under it can be inspected like any other.
    An arc that cannot be resolved is indistinguishable from one that resolves to nothing,
    and refusing every stage with a payload on ``/World`` would rule out most authored
    environments to guard against a layer that may never appear.

    Unselected variants are left to the caller. Whether one hides anything depends on what
    it defines, which this cannot see and the caller can.

    Args:
        prim: Prim to check.

    Returns:
        True if paths beneath the prim cannot be inspected.
    """
    return (prim.HasAuthoredPayloads() and not prim.IsLoaded()) or not prim.IsActive() or prim.IsInstance()


def _conceals_descendants(prim: Usd.Prim) -> bool:
    """Check whether a prim can hold content that traversing the stage will never reach.

    Composition stops at each of these, so whatever is underneath is absent from the stage
    entirely and a traversal cannot tell a subtree that is empty from one that is merely
    hidden. None of them are things this importer authors, so treating them as concealment
    costs nothing and misreading one as an empty subtree costs the user their content.

    Any authored arc counts here, whether or not it currently resolves. An arc whose target
    is missing contributes no children and leaves ``IsLoaded()`` answering True, so a prim
    carrying one is indistinguishable from a bare prim by traversal alone -- and deleting
    it destroys the arc, which is the user's work even when what it points at is absent.

    Args:
        prim: Prim to check.

    Returns:
        True if content could be hidden beneath the prim.
    """
    return (
        prim.HasAuthoredPayloads()
        or prim.HasAuthoredReferences()
        or prim.HasAuthoredInherits()
        or prim.HasAuthoredSpecializes()
        or _has_unselected_variants(prim)
        or _hides_namespace_below(prim)
    )


def is_importer_generated(stage: Usd.Stage, path: str) -> bool:
    """Check whether everything at ``path`` came from a previous heightmap generation.

    The whole subtree is checked, not just the prim itself: replacing the path discards
    everything beneath it, so a single prim of the user's own parented under generated
    terrain is enough to make the path worth warning about.

    Anything that cannot be inspected is treated as the user's, because the answer decides
    whether they get warned before it is deleted.

    Args:
        stage: USD stage to inspect.
        path: Prim path to check.

    Returns:
        True if the prim exists and it and all of its descendants carry the marker.

    Example:

    .. code-block:: python

        >>> import omni.usd
        >>> from isaacsim.asset.importer.heightmap.importer import is_importer_generated

        >>> stage = omni.usd.get_context().get_stage()  # doctest: +NO_CHECK
        >>> is_importer_generated(stage, "/World/occupancyMap")  # doctest: +NO_CHECK
        False
    """
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return False
    # Every prim, not just the ones the default predicate admits: a deactivated prim of the
    # user's would otherwise be skipped, and skipping the whole subtree makes the answer true.
    for descendant in Usd.PrimRange(prim, Usd.PrimAllPrimsPredicate):
        # Checked on every prim rather than just the root, because content concealed
        # beneath a marked descendant is lost with the subtree just the same.
        if _conceals_descendants(descendant):
            return False
        if not _has_generated_marker(descendant):
            return False
    return True


class HeightmapImporter:
    """Converts heightmap/occupancy map images into 3D terrain.

    This class handles the conversion of 2D heightmap images into 3D terrain
    in USD stages using point instancers for efficient rendering.

    Args:
        stage: USD stage to create heightmap in. If None, uses the current stage.
    """

    def __init__(self, stage: Usd.Stage | None = None) -> None:
        self._stage = stage
        self._units_per_meter = 1.0
        self._terrain_path: str | None = None
        self._muted_index: list[tuple[str, set[str] | None, Usd.Stage | None]] = []

    @property
    def terrain_path(self) -> str | None:
        """Report where the last fully successful run put the terrain.

        Not always ``OCCUPANCY_MAP_PATH``: a run that is not allowed to overwrite builds at
        the next free suffixed path instead. Only a run that finished every step sets this,
        so a failed run leaves whatever the run before it built, and None means no run on
        this importer has completed.

        Returns:
            Path of the terrain root built by the most recent successful call, or None if
            there has not been one.
        """
        return self._terrain_path

    def create_heightmap(
        self,
        image: Image.Image,
        cell_scale: float,
        create_ground_plane: bool = True,
        create_lighting: bool = True,
        configure_stage_metadata: bool = True,
        overwrite_existing: bool = True,
    ) -> int:
        """Create a 3D heightmap terrain from a heightmap image.

        This method generates heightmap terrain using point instancers for efficient
        rendering. Dark pixels in the image (below threshold) are converted to
        cube instances in 3D space.

        Args:
            image: PIL Image representing the heightmap/occupancy map.
            cell_scale: The scale of each cell in meters.
            create_ground_plane: Whether to create a ground plane for the heightmap.
            create_lighting: Whether to create lighting for the scene.
            configure_stage_metadata: Whether to set the stage's meters-per-unit and up-axis
                to the values the terrain is built for. Disable to leave a populated stage's
                own metadata untouched.
            overwrite_existing: Whether to replace whatever is at the generated paths.
                Disable to leave those paths untouched and build alongside them instead:
                the terrain goes to the next free ``occupancyMap_NN`` path, and the ground
                plane and light are skipped if theirs are taken rather than duplicated.
                Terrain built by a previous run is left alone too, so each call adds one.
                This governs prims only; stage metadata is still written unless
                ``configure_stage_metadata`` is disabled as well.

        Returns:
            The number of cells created in the heightmap. Where the terrain was built is
            left in the ``terrain_path`` attribute.

        Raises:
            ValueError: If ``image`` is None, is not a PIL image, cannot be read, or
                ``cell_scale`` is invalid.
            RuntimeError: If the USD stage is not available, if an ancestor of the generated
                paths hides them from inspection, if ``overwrite_existing`` is True and a
                prim occupying one of the paths this run writes cannot be removed, or if it
                is False and no candidate terrain path is free. USD raises
                ``pxr.Tf.ErrorException``, a subclass of this, if authoring fails for a
                reason this method did not anticipate.

        Example:

        .. code-block:: python

            >>> from PIL import Image
            >>> import omni.usd
            >>>
            >>> importer = HeightmapImporter()
            >>> image = Image.open("heightmap.png")
            >>>
            >>> num_cells = importer.create_heightmap(
            ...     image,
            ...     cell_scale=0.05,
            ... )
            ...
            >>> print(f"Created {num_cells} cells")
            Created 1234 cells
        """
        # Validate inputs
        if image is None:
            raise ValueError("Image cannot be None")

        if not isinstance(image, Image.Image):
            raise ValueError(f"Image must be a PIL Image, got {type(image).__name__}")

        if cell_scale <= 0:
            raise ValueError(f"Cell scale must be positive, got {cell_scale}")

        # Get or validate stage
        if self._stage is None:
            self._stage = omni.usd.get_context().get_stage()

        if self._stage is None:
            raise RuntimeError("No USD stage available")

        image_width, image_height = image.size

        # Provide feedback for large images
        total_pixels = image_width * image_height
        if total_pixels > LARGE_IMAGE_THRESHOLD:
            carb.log_info(f"Processing large image ({total_pixels:,} pixels), this may take a moment...")

        # GroundPlane and the stage_utils helpers resolve the current stage internally, so bind
        # the context to self._stage to keep every write on the stage the caller asked for.
        with stage_utils.use_stage(self._stage):
            # Where this run writes is settled before it writes anything, so a run that
            # cannot proceed at all stops before touching the stage.
            # Read before any path is tested, because every emptiness check consults it.
            self._index_muted_layers()
            self._verify_generated_paths_are_visible()
            if overwrite_existing:
                terrain_path = OCCUPANCY_MAP_PATH
                self._verify_generated_paths_are_clearable(create_ground_plane, create_lighting, terrain_path)
            else:
                terrain_path = self._resolve_terrain_path()
                create_ground_plane = create_ground_plane and self._may_create_furniture(GROUND_PLANE_PATH)
                create_lighting = create_lighting and self._may_create_furniture(LIGHT_PATH)

            # Setup stage properties
            carb.log_info("Setting up stage properties...")
            self._setup_stage_properties(configure_stage_metadata)
            self._units_per_meter = self._resolve_units_per_meter()

            # Each path is marked as soon as everything beneath it exists, rather than in one
            # pass at the end. A run that fails partway would otherwise leave output of its
            # own behind unmarked, and every later run would then take it for the user's:
            # refusing to build over it, and warning about replacing it. Only paths this run
            # actually wrote are marked, because stamping one it skipped would brand the
            # user's own prim as ours and silence the next run's warning about replacing it.
            if create_lighting:
                carb.log_info("Setting up lighting...")
                self._create_lighting()
                self._mark_subtree_as_generated(LIGHT_PATH)

            # Generate heightmap instances
            carb.log_info("Generating heightmap instances...")
            num_cells = self._create_heightmap_instances(image, cell_scale, terrain_path)
            self._mark_subtree_as_generated(terrain_path)
            if terrain_path != OCCUPANCY_MAP_PATH:
                carb.log_warn(
                    f"{OCCUPANCY_MAP_PATH} is already taken, so the terrain was generated at "
                    f"{terrain_path} instead. Enable 'Overwrite Existing' to replace what is "
                    f"at {OCCUPANCY_MAP_PATH} rather than building alongside it."
                )

            # Create ground plane sized to the heightmap's actual XY bounds (after instances exist)
            if create_ground_plane:
                carb.log_info("Creating ground plane...")
                if self._create_ground_plane(terrain_path):
                    self._mark_subtree_as_generated(GROUND_PLANE_PATH)

            # Every step is done, so the run can finally say where it put the terrain.
            self._terrain_path = terrain_path

        carb.log_info(f"Heightmap generation complete! Created {num_cells} cells.")

        return num_cells

    def _verify_generated_paths_are_clearable(
        self, create_ground_plane: bool, create_lighting: bool, terrain_path: str
    ) -> None:
        """Check up front that every path this run writes to can be cleared.

        Only reached when the caller insists on these exact paths. A run allowed to choose
        its own path never needs this, because it only picks paths it can write.

        Args:
            create_ground_plane: Whether the run creates a ground plane.
            create_lighting: Whether the run creates a light.
            terrain_path: Where the terrain itself goes.

        Raises:
            RuntimeError: If an existing prim cannot be removed from the current edit target.
        """
        for path in generated_paths(create_ground_plane, create_lighting, terrain_path):
            surviving_specs = self._surviving_specs_at(path)
            if surviving_specs:
                raise RuntimeError(
                    f"Cannot generate terrain: {path} is also defined at {', '.join(sorted(set(surviving_specs)))}, "
                    "which the layer being edited cannot remove. Move that prim aside, or edit the "
                    "layer that defines it."
                )

    def _concealing_ancestor_of(self, path: str) -> str | None:
        """Find an ancestor of ``path`` that hides whatever sits beneath it.

        Composition stops at an inactive prim, an unloaded payload, an instance boundary
        and an unselected variant, so a path under one of those resolves to nothing even
        when a prim of the user's is authored there. Reading that as an empty path is how a
        run forbidden to overwrite ends up stacking its terrain on their content anyway.

        A loaded payload is not among them. Its content is composed into the namespace, so
        paths beneath it can be inspected like any other. Nor is a variant set that leaves
        the generated paths alone: a lookdev or time-of-day set on ``/World`` hides nothing
        this run cares about, and refusing it would rule out the stages this option exists
        to generate into.

        Args:
            path: Prim path whose ancestors to examine.

        Returns:
            Path of the nearest concealing ancestor, or None when every ancestor is clear.
        """
        variants_hide_a_path_we_want = _a_variant_could_claim_a_generated_path(self._stage)
        ancestor = Sdf.Path(path).GetParentPath()
        while not ancestor.isEmpty and ancestor != Sdf.Path.absoluteRootPath:
            prim = self._stage.GetPrimAtPath(ancestor)
            if prim.IsValid() and (
                _hides_namespace_below(prim) or (variants_hide_a_path_we_want and _has_unselected_variants(prim))
            ):
                return ancestor.pathString
            ancestor = ancestor.GetParentPath()
        return None

    def _verify_generated_paths_are_visible(self) -> None:
        """Refuse to generate while anything hides the paths this run would write to.

        Checked once against the terrain path, which is enough: every path this importer
        writes is a child of ``WORLD_PATH``, so they all share the same ancestors.

        Neither mode can proceed. Overwriting cannot report what it is about to replace,
        and stepping aside cannot find a path that is genuinely free, because every
        candidate is hidden by the same ancestor.

        Raises:
            RuntimeError: If an ancestor of the generated paths conceals its descendants.
        """
        concealed_by = self._concealing_ancestor_of(OCCUPANCY_MAP_PATH)
        if concealed_by is None:
            return
        raise RuntimeError(
            f"Cannot generate terrain: {concealed_by} hides everything beneath it, so the "
            "generated paths cannot be checked for content of your own. Activate that prim, "
            "load its payload, or select the variant that defines it, then generate again."
        )

    def _surviving_specs_at(self, path: str) -> list[str]:
        """Find the specs at ``path`` that clearing it would leave behind.

        :py:meth:`pxr.Usd.Stage.RemovePrim` only removes the spec the edit target maps to.
        Any other spec contributing to the prim — one in a sublayer, or one authored inside a
        variant of the same layer — survives, and redefining the path would then stack a
        second set of transform ops onto the old prim or silently reuse it.

        Muted layers are included. Their specs contribute nothing while muted, so the prim
        does not even resolve, but unmuting brings them back alongside whatever was written
        over the top.

        Args:
            path: Prim path to examine.

        Returns:
            Descriptions of the specs that would survive clearing the path.
        """
        surviving = self._muted_specs_at(path)
        prim = self._stage.GetPrimAtPath(path)
        if prim.IsValid():
            edit_target = self._stage.GetEditTarget()
            edit_layer = edit_target.GetLayer()
            # Comparing the whole spec location, not just the layer, so that a spec authored
            # in a variant of the edit layer is recognized as one RemovePrim cannot reach.
            removable_spec_path = edit_target.MapToSpecPath(path)
            surviving += [
                f"{spec.layer.identifier}{spec.path}"
                for spec in prim.GetPrimStack()
                if spec.layer != edit_layer or spec.path != removable_spec_path
            ]
        return surviving

    def _is_free(self, path: str) -> bool:
        """Check whether ``path`` is empty, so writing there destroys nothing.

        A run that was not given permission to overwrite takes this literally: output of a
        previous run counts as occupied just as much as the user's own content does. The
        alternative, quietly reusing paths this importer happens to recognize, means an
        option labeled as being about overwriting still overwrites things.

        Args:
            path: Prim path to test.

        Returns:
            True if nothing is defined at the path.
        """
        if self._stage.GetPrimAtPath(path).IsValid():
            return False
        # Still not free if a muted layer defines it: unmuting would stack that spec
        # underneath whatever was written on top.
        return not self._muted_specs_at(path)

    def _may_create_furniture(self, path: str) -> bool:
        """Check whether the ground plane or light at ``path`` should be created at all.

        These two are not given suffixed paths the way the terrain is. A second ground plane
        would sit coincident with the one already there and z-fight with it, and a second
        distant light would stack onto whatever lighting the scene already has — the outcome
        the option exists to avoid. So when the path is taken, nothing is created and the
        scene keeps what it had, whoever put it there.

        Args:
            path: Ground plane or light path this run would write.

        Returns:
            True if the run may create it.
        """
        if self._is_free(path):
            return True
        carb.log_warn(
            f"{path} is already taken, so nothing was generated there and the scene keeps what "
            "it had. Enable 'Overwrite Existing' to replace it instead."
        )
        return False

    def _resolve_terrain_path(self) -> str:
        """Pick a free path for the terrain, leaving every terrain already built alone.

        The unsuffixed path is preferred, so the first run on a clean stage lands at
        ``/World/occupancyMap``. Each run after that steps to the next free suffix rather
        than replacing what the last one built, which is what lets a stage hold more than
        one terrain and what keeps this option from overwriting anything.

        Returns:
            The path to build the terrain at.

        Raises:
            RuntimeError: If no candidate path is free.
        """
        for index in range(MAX_TERRAIN_PATH_CANDIDATES):
            candidate = OCCUPANCY_MAP_PATH if index == 0 else f"{OCCUPANCY_MAP_PATH}_{index:02d}"
            if self._is_free(candidate):
                return candidate
        raise RuntimeError(
            f"Cannot generate terrain: {OCCUPANCY_MAP_PATH} and the next "
            f"{MAX_TERRAIN_PATH_CANDIDATES - 1} suffixed paths are all occupied. Delete some of "
            "them, or enable 'Overwrite Existing' to reuse the first path."
        )

    def _index_muted_layers(self) -> None:
        """Read what the muted layers define, once, before any path is tested.

        Path resolution asks about up to :py:data:`MAX_TERRAIN_PATH_CANDIDATES` paths, and
        answering each one from scratch would re-walk and re-compose every muted layer that
        many times. Everything expensive happens here instead, and the per-path question
        becomes a set lookup plus a composed lookup on an already-open stage.
        """
        self._muted_index = []
        for identifier in self._stage.GetMutedLayers():
            root = Sdf.Layer.FindOrOpen(identifier)
            if root is None:
                # Unreadable, so nothing about it can be ruled out.
                self._muted_index.append((identifier, None, None))
                continue
            # Composed, because arcs contribute paths no layer's own specs mention.
            # Payloads stay unloaded: a muted layer can be a whole environment, and all
            # that hangs on the answer is whether to suffix. What an unloaded payload
            # might define is handled per path, by treating its ancestor as occupied.
            stage = Usd.Stage.Open(root, Usd.Stage.LoadNone)
            self._muted_index.append((identifier, _prim_spec_paths(stage), stage))

    def _muted_specs_at(self, path: str) -> list[str]:
        """Find specs at ``path`` in layers that are currently muted.

        Muted layers are absent from the composed stage and from
        :py:meth:`pxr.Usd.Prim.GetPrimStack`, so they have to be looked up directly.
        Writing where one of them defines a prim stacks this run's output on content that
        comes back the moment the layer is unmuted.

        Args:
            path: Prim path to look for.

        Returns:
            Identifiers of the muted layers holding a spec at the path.
        """
        specs = []
        for identifier, spec_paths, stage in self._muted_index:
            if (
                spec_paths is None
                or path in spec_paths
                or stage.GetPrimAtPath(path).IsValid()
                or _has_unloaded_payload_above(stage, path)
            ):
                specs.append(f"{identifier}{path} (muted)")
        return specs

    def _clear_prim(self, path: str) -> None:
        """Remove any prim at ``path`` so it can be defined from scratch.

        Args:
            path: Prim path to clear.

        Raises:
            RuntimeError: If a prim exists at the path but could not be removed.
        """
        if not self._stage.GetPrimAtPath(path).IsValid():
            return
        if not self._stage.RemovePrim(path):
            raise RuntimeError(f"Could not remove the existing prim at {path}.")

    def _mark_subtree_as_generated(self, path: str) -> None:
        """Stamp the importer's marker attribute across everything it created at ``path``.

        Descendants are marked too, including the ones built for us by ``GroundPlane``, so
        that a later run can recognize an untouched subtree by the absence of any unmarked
        prim in it.

        Args:
            path: Root prim path of the generated content to mark.
        """
        prim = self._stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        # Same traversal :py:func:`is_importer_generated` reads with, so that everything it
        # will later inspect is something this marked.
        for descendant in Usd.PrimRange(prim, Usd.PrimAllPrimsPredicate):
            descendant.CreateAttribute(GENERATED_MARKER_ATTR, Sdf.ValueTypeNames.Bool, custom=True).Set(True)

    def _setup_stage_properties(self, configure_stage_metadata: bool = True) -> None:
        """Ensure the world prim exists, and optionally set stage units and up-axis.

        Args:
            configure_stage_metadata: Whether to overwrite the stage's meters-per-unit,
                up-axis, and default prim. The world prim is created either way, since
                the terrain is parented under it.
        """
        if configure_stage_metadata:
            UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
            stage_utils.set_stage_up_axis("Z")

        world_prim = self._stage.GetPrimAtPath(WORLD_PATH)
        if not world_prim.IsValid():
            UsdGeom.Xform.Define(self._stage, WORLD_PATH)
            world_prim = self._stage.GetPrimAtPath(WORLD_PATH)

        if configure_stage_metadata:
            self._stage.SetDefaultPrim(world_prim)

    def _resolve_units_per_meter(self) -> float:
        """Return the factor that converts a length in meters into the stage's units.

        Cell size and cube height are specified in meters, so authoring them directly as
        stage units would make terrain the wrong physical size on a stage that is not
        metric.

        Returns:
            Stage units per meter. 1.0 for a metric stage.
        """
        meters_per_unit = UsdGeom.GetStageMetersPerUnit(self._stage)
        # Phrased as a failed positive test so that NaN is rejected too.
        if not meters_per_unit > 0.0:
            carb.log_warn(f"Stage reports {meters_per_unit} meters per unit; generating as if it were metric.")
            return 1.0
        return 1.0 / meters_per_unit

    def _create_ground_plane(self, terrain_path: str) -> bool:
        """Create a ground plane sized to fit the heightmap's XY bounds.

        Must be called after :py:meth:`_create_heightmap_instances` so the bounding box
        computed from the terrain prim reflects the generated geometry.

        Args:
            terrain_path: Root of the terrain this run generated, which the plane is sized
                to. Terrain generated at some other path keeps no floor of its own.

        Returns:
            True if a ground plane was created. False when the terrain is empty and there is
            nothing to put a floor under, in which case only a floor from a previous run is
            cleared and anything the user authored at the path is left alone.
        """
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        bounds = bbox_cache.ComputeWorldBound(self._stage.GetPrimAtPath(terrain_path)).GetRange()
        if bounds.IsEmpty():
            carb.log_warn("Occupancy map has empty bounds; skipping ground plane creation.")
            # A floor left over from a previous run would outlive the terrain it was sized
            # for. Only our own is cleared, so a floor of the user's own survives.
            if is_importer_generated(self._stage, GROUND_PLANE_PATH):
                self._clear_prim(GROUND_PLANE_PATH)
            return False
        min_pt, max_pt = bounds.GetMin(), bounds.GetMax()

        # Use the longer XY dimension so a square plane fully covers the heightmap without stretching.
        margin = GROUND_PLANE_MARGIN * self._units_per_meter
        size = max(max_pt[0] - min_pt[0], max_pt[1] - min_pt[1]) + 2 * margin
        center_x = (min_pt[0] + max_pt[0]) / 2.0
        center_y = (min_pt[1] + max_pt[1]) / 2.0

        # GroundPlane wraps an existing prim instead of rebuilding it, and ignores sizes when it
        # does, so clear the path first or a regenerated terrain keeps the first run's floor.
        self._clear_prim(GROUND_PLANE_PATH)
        GroundPlane(
            GROUND_PLANE_PATH,
            sizes=size,
            positions=[[center_x, center_y, 0.0]],
            colors=[1.0, 1.0, 1.0],
        )
        return True

    def _create_lighting(self) -> None:
        """Create a distant light for the scene."""
        # DistantLight.Define retypes an existing prim in place, keeping its children and any
        # attributes it does not set, so clear the path to get a light with no leftovers.
        self._clear_prim(LIGHT_PATH)
        light_prim = UsdLux.DistantLight.Define(self._stage, Sdf.Path(LIGHT_PATH))
        light_prim.CreateIntensityAttr(DEFAULT_LIGHT_INTENSITY)

    def _create_heightmap_instances(self, image: Image.Image, cell_scale: float, terrain_path: str) -> int:
        """Create the point instancer with cube instances for occupied cells.

        Args:
            image: PIL Image representing the heightmap/occupancy map.
            cell_scale: The scale of each cell in meters.
            terrain_path: Root to build the terrain under.

        Returns:
            The number of cells created.
        """
        cell_size = cell_scale * self._units_per_meter
        cell_offset = cell_size / 2.0

        # Read the image first. It is the only fallible step here, and doing it before any
        # prim exists means a bad image leaves no half-built terrain behind to be mistaken
        # for the user's content by every run that follows.
        occupied_positions = self._generate_occupied_positions(image, cell_size, cell_offset)

        try:
            self._create_parent_transform(cell_offset, terrain_path)
            point_instancer = self._create_point_instancer(terrain_path)
            cube_prototype = self._create_cube_prototype(cell_size, terrain_path)
            self._configure_point_instancer(point_instancer, cube_prototype, occupied_positions)
        except Exception:
            # The marker goes on once the subtree is complete, so prims written before a
            # failure here carry none, and every later run would read them as the user's:
            # building alongside them instead of over them, and warning before replacing
            # them. Removing them leaves the path as this run found it.
            self._clear_prim(terrain_path)
            raise

        return len(occupied_positions)

    def _create_parent_transform(self, cell_offset: float, terrain_path: str) -> UsdGeom.Xform:
        """Create the parent transform for the terrain.

        Args:
            cell_offset: The offset to center cells at their grid positions.
            terrain_path: Path to define the transform at.

        Returns:
            The USD Xform primitive for the parent transform.
        """
        self._clear_prim(terrain_path)
        parent_xform = UsdGeom.Xform(self._stage.DefinePrim(terrain_path, "Xform"))
        parent_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, cell_offset))
        return parent_xform

    def _create_point_instancer(self, terrain_path: str) -> UsdGeom.PointInstancer:
        """Create the point instancer that will contain all cube instances.

        Args:
            terrain_path: Root of the terrain the instancer goes under.

        Returns:
            The USD PointInstancer primitive.
        """
        path = instances_path(terrain_path)
        self._clear_prim(path)
        cube_height = DEFAULT_CUBE_HEIGHT * self._units_per_meter
        point_instancer = UsdGeom.PointInstancer(self._stage.DefinePrim(path, "PointInstancer"))
        point_instancer.AddScaleOp().Set(Gf.Vec3f(1))
        point_instancer.AddTranslateOp().Set(Gf.Vec3f(0, 0, (cube_height / 2.0)))
        return point_instancer

    def _configure_point_instancer(
        self, point_instancer: UsdGeom.PointInstancer, cube_prototype: UsdGeom.Cube, positions: list[Gf.Vec3f]
    ) -> None:
        """Configure the point instancer with positions and prototype.

        Args:
            point_instancer: The point instancer to configure.
            cube_prototype: The cube prototype to instance.
            positions: List of positions for each instance.
        """
        if not positions:
            carb.log_warn("No occupied positions found. Heightmap will be empty.")
            return

        point_instancer.CreatePositionsAttr().Set(positions)
        point_instancer.CreatePrototypesRel().SetTargets([cube_prototype.GetPath()])
        point_instancer.CreateProtoIndicesAttr().Set([0] * len(positions))

    def _create_cube_prototype(self, cell_size: float, terrain_path: str) -> UsdGeom.Cube:
        """Create the cube prototype that will be instanced for each occupied cell.

        Args:
            cell_size: The size of each cell in stage units.
            terrain_path: Root of the terrain the prototype goes under.

        Returns:
            The USD cube primitive to be used as a prototype.
        """
        path = cube_prototype_path(terrain_path)
        self._clear_prim(path)

        cube_height = DEFAULT_CUBE_HEIGHT * self._units_per_meter
        cube = UsdGeom.Cube(self._stage.DefinePrim(path, "Cube"))
        cube.AddScaleOp().Set(Gf.Vec3f(cell_size, cell_size, cube_height))
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorPrimvar().Set([DEFAULT_CUBE_COLOR])
        UsdPhysics.CollisionAPI.Apply(self._stage.GetPrimAtPath(path))

        return cube

    def _generate_occupied_positions(self, image: Image.Image, cell_size: float, cell_offset: float) -> list[Gf.Vec3f]:
        """Generate 3D positions for all occupied cells in the image.

        Uses NumPy for efficient vectorized processing of large images.

        Args:
            image: PIL Image representing the heightmap/occupancy map.
            cell_size: The size of each cell in stage units.
            cell_offset: The offset to center cells at their grid positions.

        Returns:
            List of Gf.Vec3f positions for each occupied cell.
        """
        try:
            # Convert PIL Image to numpy array for fast processing
            img_array = np.array(image)

            # Handle 2D grayscale (mode 'L', 'I', 'F') and 3D+ (RGB/RGBA) arrays
            if img_array.ndim == 2:
                channel = img_array
            elif img_array.ndim >= 3 and img_array.shape[2] >= 1:
                channel = img_array[:, :, 0]
            else:
                raise ValueError(
                    f"Image has invalid shape: {img_array.shape}. Expected 2D grayscale or 3D+ with channels."
                )

            # Find occupied pixels (below threshold)
            occupied_mask = channel < OCCUPIED_PIXEL_THRESHOLD

            # Get coordinates of occupied pixels (y, x order from numpy)
            y_coords, x_coords = np.where(occupied_mask)

            # Vectorized position calculation
            world_x = (x_coords * cell_size) + cell_offset
            world_y = -((y_coords * cell_size) + cell_offset)

            # Create list of Vec3f positions
            occupied_positions = [Gf.Vec3f(float(x), float(y), 0.0) for x, y in zip(world_x, world_y)]

            return occupied_positions
        except IndexError as e:
            raise ValueError(f"Failed to access image array channels: {e}") from e
        except ValueError:
            # Already the documented type, and already carries a message about the image.
            raise
        except Exception as e:
            raise ValueError(f"Failed to generate occupied positions: {e}") from e
