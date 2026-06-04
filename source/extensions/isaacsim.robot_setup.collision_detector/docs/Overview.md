# Overview

A dockable panel for enumerating self-collision pairs in robot articulations
and managing collision filters.

## Usage

1. Open a stage containing one or more robots (with `RobotAPI` applied).
2. The panel auto-discovers all robots on the stage and populates a dropdown.
   Select the target robot from the **Robot** dropdown.
3. Click **Check Collisions** to enumerate collision pairs and display them
   in the TreeView.
4. Optionally enable **Include environment collisions** before checking to
   also list pairs between robot links and non-robot bodies on the stage.

## Features

* **Robot auto-discovery** – scans the stage for prims with the Isaac
  `RobotAPI` schema and presents them in a dropdown. Automatically
  refreshes when robots are added or removed (via USD notice listener).
* **Check Collisions** – uses the physics engine's initial-collider-pair
  query to enumerate colliding rigid body pairs under the selected robot
  and merges them with any existing `FilteredPairsAPI` relationships.
* **Include environment collisions** – checkbox that broadens the pair
  enumeration to include collisions between robot links and non-robot
  bodies in the scene.
* **Rigid Body A / B columns** – each body cell shows a colour swatch
  (unique per body, generated from a 64-colour perceptually-distinct
  palette) and the body prim name. Per-column A-Z / Z-A sorting via
  header sort icons.
* **Filtered Pair column** – checkbox per row to toggle
  `UsdPhysics.FilteredPairsAPI` between the two bodies. Supports batch
  toggling across multi-selected rows.
* **Search** – free-text search field that filters the pair list by body
  name in real time.
* **Select Collision Prim** – per-body focal icon that selects the
  `CollisionAPI` prim(s) under that body in the Stage window.
* **Viewport highlight** – selecting a row assigns selection-group
  outline and shade colours to both bodies and applies non-destructive
  `displayColor` / `displayOpacity` overrides via the session layer.
  Deselecting clears all overlays.
* **Stage awareness** – resets state on stage open/close; clears viewport
  overlays when the panel is hidden.


```{image} ../../../../source/extensions/isaacsim.robot_setup.collision_detector/data/preview.png
---
align: center
---
```
