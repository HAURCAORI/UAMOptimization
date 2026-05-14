# CODEX_INSTRUCTIONS_VISUALIZATION.md

## Visualization Integration Instructions

This section defines the implementation instructions for adding a **basic 3D visualization module** to `FrameworkCpp` using the existing `VulkanTemplate` codebase as a **reference source**, not as a full engine to be copied wholesale.

The goal is **not** to build a full rendering engine.  
The goal is to provide a simple viewer that allows the user to **visually verify the placement and assembly of spatial elements** in the current Stage 1 architecture framework.

---

## Scope

The visualization work is limited to:

- reading the current assembled `HexacopterArchitecture`,
- extracting world-space primitive placement from `AssemblyState`,
- rendering those primitives in 3D,
- providing a basic camera,
- optionally showing minimal UI/debug info.

This viewer is for **visual validation of placement**, not for simulation, animation, Stage 2 mission playback, FEM, CAD import, or advanced rendering.

---

## Hard Rules

1. **Do not merge the whole `VulkanTemplate` engine into `FrameworkCpp`.**
   - Reuse only the minimal Vulkan/bootstrap/render-loop/camera/UI logic needed.

2. **Do not put visualization code into `core/`, `physics/`, `evaluation/`, or `optimization/`.**
   - Visualization must live in a **dedicated new folder**.

3. **Do not make visualization a dependency of the architecture model.**
   - The viewer consumes `AssemblyState` and `GeometryPrimitive`.
   - The core framework must not depend on Vulkan.

4. **Do not implement advanced features.**
   - No shadows
   - No PBR
   - No textures/material system
   - No postprocess
   - No culling pipeline
   - No scene loader
   - No mesh/CAD import

5. **Keep the viewer primitive-based.**
   - The viewer must render the framework's existing reduced-order primitives only.

---

## What to Read First

Codex must first read and understand the structure of the provided `VulkanTemplate` code.

Required understanding targets:
- Vulkan context / device setup
- swapchain / frame loop
- command buffer submission flow
- camera handling
- UI layer structure
- shader / pipeline setup style

Codex must also read the relevant `FrameworkCpp` architecture interfaces:

- `HexacopterArchitecture`
- `AssemblyState`
- `GeometryPrimitive`
- public element model
- current app entry style

The viewer must be designed around the **actual assembled graph** of `FrameworkCpp`, not around a separate scene graph invented inside the viewer.

---

## Target Folder Structure

Add a dedicated visualization module under `FrameworkCpp/`:

```text
FrameworkCpp/
  app/
    main.cpp
    main_visualizer.cpp

  visualization/
    include/visualization/
      PrimitiveInstance.hpp
      ArchitectureSceneBuilder.hpp
      PrimitiveMeshFactory.hpp
      ViewerCamera.hpp
      ArchitectureViewerApp.hpp
    src/visualization/
      PrimitiveInstance.cpp
      ArchitectureSceneBuilder.cpp
      PrimitiveMeshFactory.cpp
      ViewerCamera.cpp
      ArchitectureViewerApp.cpp
    shaders/
      primitive.vert
      primitive.frag
```

If some helper files are better grouped differently, minor adjustment is acceptable, but the visualization code must remain **isolated** in its own module.

---

## Required Integration Strategy

## 1. Use `VulkanTemplate` as a reference, not as a direct transplant

Reuse only the minimum ideas/patterns from `VulkanTemplate`, such as:

- context/device creation
- swapchain setup
- frame loop
- per-frame command recording
- camera matrix handling
- minimal ImGui integration if already easy to port
- basic graphics pipeline setup

Do **not** import the full template architecture:
- no general scene system
- no world/asset loader
- no material framework
- no shadow or postprocess system
- no complex render passes beyond what the simple primitive viewer needs

The result should feel like a **small custom viewer built using lessons from the template**, not like the template was force-fit into `FrameworkCpp`.

---

## 2. Visualization must consume `AssemblyState`

The viewer must not reconstruct geometry from raw scalar parameters if the assembled graph already provides the information.

Use:
- `HexacopterArchitecture`
- `HexacopterArchitecture::assemblyState()`
- assembled element world poses
- assembled world primitives

The visualizer must render what the framework already assembled.

This is the most important integration boundary.

---

## Required Runtime Behavior

The basic viewer must:

1. Construct the default architecture or another selected architecture instance.
2. Call `rebuildAssembly()`.
3. Extract assembled primitives.
4. Convert them into renderable primitive instances.
5. Render them in 3D with a simple camera.
6. Let the user inspect placement visually.

Optional but recommended:
- small UI panel with element count / primitive count
- wireframe toggle
- camera reset
- type-based visibility toggles

---

## Primitive Types to Support

The viewer must support the current reduced-order primitive set already used by `FrameworkCpp`:

- sphere
- box
- cylinder
- disk
- segment

These are sufficient for the initial visualization goal.

If a primitive does not yet have exact mesh support, a reasonable simple approximation is acceptable in the first implementation.

Examples:
- segment may be rendered as a thin cylinder or capsule-like line proxy
- disk may be rendered as a very thin cylinder

---

## Visualization Data Model

Add a simple visualization-side data model.

## `PrimitiveInstance`
Represents one renderable primitive in world space.

Suggested fields:
- primitive type
- world transform
- size parameters
- color
- source element id
- source element type
- optional wireframe/solid flags

This must be independent of Vulkan-specific buffer handles at first.

## `ArchitectureSceneBuilder`
This class converts framework-side assembly data into a list of `PrimitiveInstance` objects.

Input:
- `const HexacopterArchitecture&`

Output:
- `std::vector<PrimitiveInstance>`

Responsibilities:
- walk the assembled elements
- read world-space primitives
- assign a display color by element type
- generate the primitive instance list used by the renderer

This class is the bridge between `FrameworkCpp` core data and the visualization layer.

---

## Mesh Generation Strategy

Add `PrimitiveMeshFactory` to generate reusable unit meshes for:

- box
- sphere
- cylinder
- disk
- segment proxy

Recommended approach:
- generate simple procedural meshes in C++
- keep them static/reusable
- draw multiple instances using transform data

Do not add asset importing for this phase.

---

## Rendering Strategy

Keep the renderer simple.

### Required rendering style
- solid color rendering
- basic lighting or unlit shading
- one pipeline is enough if practical
- one mesh per primitive type
- one instance transform per rendered primitive

### Recommended first version
- CPU-side primitive instance list
- upload per-frame instance buffer or simple per-draw uniforms
- draw one primitive type at a time

This is enough for placement verification.

Do not optimize aggressively at this stage.

---

## Camera and Controls

The viewer must support a basic interactive 3D camera.

Minimum controls:
- orbit
- pan
- zoom

A simple perspective camera is sufficient.

Recommended behavior:
- initialize camera to frame the entire assembled model
- support camera reset
- optionally support focus-on-origin

If `VulkanTemplate` already has reusable camera logic, adapt the minimal required subset.

---

## UI / Debug Panel

Advanced UI is not required.

If easy to integrate, add a very small ImGui panel with:
- primitive count
- element count
- wireframe toggle
- camera reset
- type visibility toggles

Do not build a complex inspector unless it is nearly free.

---

## Build / Dependency Policy

The visualization integration must not destroy the current `FrameworkCpp` build structure.

### Rules
- keep `FrameworkCpp` as the main project
- add the visualization module as an optional component/executable
- do not replace the current build system with the raw `VulkanTemplate` build system
- do not blindly copy the template CMake hierarchy

Codex should adapt the useful rendering code patterns into the existing project layout.

If additional Vulkan-related libraries or setup are needed, keep them isolated to the visualization target.

---

## Phase-by-Phase Implementation Plan

## Phase V0 — read and plan
Before changing code:
1. Inspect `VulkanTemplate` structure.
2. Identify the smallest reusable Vulkan bootstrap pieces.
3. Inspect `FrameworkCpp` architecture/assembly interfaces.
4. Confirm the visualization boundary:
   - input = assembled architecture data
   - output = primitive viewer

No code generation should begin before this understanding step.

---

## Phase V1 — add visualization skeleton
Create the new visualization folder structure and empty/basic files:

- `PrimitiveInstance`
- `ArchitectureSceneBuilder`
- `PrimitiveMeshFactory`
- `ViewerCamera`
- `ArchitectureViewerApp`
- `app/main_visualizer.cpp`

At this stage:
- compile stubs if possible
- no advanced rendering features

---

## Phase V2 — scene bridge only
Implement `ArchitectureSceneBuilder` first.

This phase must:
- read `AssemblyState`
- extract all world primitives
- assign colors by element type
- return a clean list of primitive instances

Before rendering, add temporary logging/debug output to verify:
- primitive count
- per-type counts
- world transforms exist
- sizes are correct

This phase is important because it validates the framework-to-viewer bridge independently of Vulkan.

---

## Phase V3 — minimal Vulkan viewer
Implement the actual viewer using the minimum Vulkan subset.

Required features:
- window creation
- Vulkan context/device
- swapchain
- frame loop
- render pass
- basic pipeline
- camera matrices
- draw primitive meshes from instance data

No advanced render graph.

Goal:
- show all assembled primitives in one 3D scene

---

## Phase V4 — small usability improvements
After the scene renders correctly, add optional small features:
- wireframe toggle
- element-type visibility toggles
- simple UI panel
- camera reset

Stop here unless the user explicitly asks for more.

---

## File-Level Responsibilities

### `PrimitiveInstance.hpp`
Defines the renderer-facing primitive instance description.

### `ArchitectureSceneBuilder.hpp/.cpp`
Bridges `FrameworkCpp` assembly data to visualization instance data.

### `PrimitiveMeshFactory.hpp/.cpp`
Creates reusable unit meshes.

### `ViewerCamera.hpp/.cpp`
Basic orbit/pan/zoom camera and view/projection matrices.

### `ArchitectureViewerApp.hpp/.cpp`
Owns the viewer runtime:
- init
- frame loop
- GPU resources
- rendering
- optional UI

### `app/main_visualizer.cpp`
Standalone executable entry point for visualization.

This must not replace the current CLI app.  
It should be a new executable or clearly separated new mode.

---

## Color Policy

Assign stable colors by element type.

Recommended simple default mapping:
- body: dark gray
- battery: blue
- payload: orange
- arm: medium gray
- motor: dark red
- rotor: green

Exact color choice is flexible, but the mapping should be stable and visually distinct.

---

## What Not to Implement

Do not add any of the following in this task:

- Stage 2 animation
- mission trajectory playback
- FEM mesh rendering
- arbitrary CAD import
- material/texture system
- physically based rendering
- shadow mapping
- post-processing
- asset database
- scene serialization framework
- render graph framework

This visualization task is only for **placement verification**.

---

## Required Acceptance Criteria

The visualization integration is acceptable only if all of the following are true:

1. `FrameworkCpp` retains its layered architecture and visualization stays isolated.
2. A dedicated visualization executable or clearly isolated visualization entry exists.
3. The visualizer reads `AssemblyState` rather than rebuilding geometry separately.
4. The viewer renders the current primitive set:
   - sphere
   - box
   - cylinder
   - disk
   - segment
5. The user can visually inspect the assembled model in 3D.
6. Camera movement is sufficient for manual verification.
7. No advanced rendering systems were unnecessarily imported from the template.

---

## Expected Deliverable

The final result should be:

- a new visualization module under `FrameworkCpp/visualization/`
- a viewer executable entry
- minimal shaders
- a working path from architecture assembly to primitive rendering
- enough UI/camera support to inspect the design visually

This is not a rendering-engine project.  
It is a **framework visualization utility**.

---

## Reporting Requirement

After implementation, report clearly:

- which files were added
- which files from `VulkanTemplate` were conceptually reused
- which template subsystems were intentionally **not** imported
- whether the viewer is a separate executable or an extra app mode
- any build/dependency assumptions that still require user action
