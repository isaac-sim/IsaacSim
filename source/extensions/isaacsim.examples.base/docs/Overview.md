# Overview

The isaacsim.examples.base extension provides foundational base classes for creating structured Isaac Sim simulation examples. It establishes a standardized framework that combines simulation logic with user interface templates, enabling developers to build consistent and reusable simulation demonstrations.

## Key Components

### {class}`BaseSample <isaacsim.examples.base.BaseSample>`

The {class}`BaseSample <isaacsim.examples.base.BaseSample>` class serves as the abstract foundation for simulation examples. It manages the complete simulation lifecycle including world loading, resetting, and cleanup operations through a structured async workflow.

The class establishes default simulation settings with a physics timestep of 1/60 seconds, stage units in meters, and rendering timestep of 1/60 seconds. It provides core simulation management including timeline control and physics simulation interface integration.

**Core lifecycle methods that subclasses must implement:**
- `setup_scene()`: Configure the simulation scene and add objects
- `setup_post_load()`: Initialize variables after world loading  
- `setup_pre_reset()`: Perform cleanup before simulation reset
- `setup_post_reset()`: Configure state after simulation reset
- `setup_post_clear()`: Handle cleanup after clearing the scene

The class automatically handles viewport camera positioning, physics scene creation, and timeline control during world loading and reset operations.

### {class}`BaseSampleUITemplate <isaacsim.examples.base.BaseSampleUITemplate>`

The {class}`BaseSampleUITemplate <isaacsim.examples.base.BaseSampleUITemplate>` class provides a standardized UI framework for Isaac Sim examples with common controls for world loading, resetting, and timeline management. It creates a collapsible frame structure with default world controls and extensible areas for custom UI elements.

The template automatically manages stage and timeline event subscriptions and button state management. It provides abstract methods for customizing behavior during different operations:

- `build_extra_frames()`: Add sample-specific UI elements
- `post_load_button_event()`: Handle actions after Load World button
- `post_reset_button_event()`: Handle actions after Reset button  
- `post_clear_button_event()`: Handle cleanup after timeline stop events

## Integration

The extension integrates with core Isaac Sim systems through several dependencies. It uses `isaacsim.core.simulation_manager` for simulation lifecycle management and `isaacsim.core.rendering_manager` for rendering control. The `isaacsim.gui.components` dependency provides UI building blocks for the template system, while `**omni.physics**` enables physics simulation capabilities within the sample framework.

## Usage Examples

```python
from isaacsim.examples.base import BaseSample, BaseSampleUITemplate

class MySimulationSample(BaseSample):
    def setup_scene(self):
        # Add objects and configure the scene
        pass
    
    async def setup_post_load(self):
        # Initialize variables after world loading
        pass
    
    async def setup_pre_reset(self):
        # Cleanup before reset
        pass
    
    async def setup_post_reset(self):
        # Configure state after reset
        pass
    
    async def setup_post_clear(self):
        # Handle cleanup after clearing
        pass

class MySimulationUI(BaseSampleUITemplate):
    def build_extra_frames(self):
        # Add custom UI elements
        pass
    
    def post_load_button_event(self):
        # Handle load button actions
        pass
    
    def post_reset_button_event(self):
        # Handle reset button actions
        pass
    
    def post_clear_button_event(self):
        # Handle clear button actions
        pass
```
