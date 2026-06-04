# Overview

isaacsim.gui.components provides a comprehensive collection of UI builder functions and wrapper classes for creating standardized Isaac Sim user interfaces. The extension offers both functional builder APIs and object-oriented wrapper classes to construct common UI elements with consistent styling and behavior across Isaac Sim extensions.

## Functionality

### Builder Functions

The extension provides builder functions that create stylized UI elements with minimal code. These functions handle styling, layout, and common configurations automatically:

```python
# Create a button with label and callback
button = btn_builder(label="Action", text="Execute", on_clicked_fn=my_callback)

# Create a dropdown with predefined options
dropdown = dropdown_builder(label="Mode", items=["Option 1", "Option 2"], on_clicked_fn=handle_selection)

# Create input fields with validation
int_field = int_builder(label="Count", default_val=10, min=0, max=100)
float_field = float_builder(label="Speed", default_val=1.0, step=0.1, format="%.2f")
```

### UI Element Wrappers

Object-oriented wrapper classes provide more advanced functionality and state management:

```python
# Create wrapper objects with enhanced capabilities
button = Button(label="Control", text="Start", on_click_fn=start_simulation)
state_btn = StateButton(label="Mode", a_text="Play", b_text="Stop", 
                       on_a_click_fn=play_fn, on_b_click_fn=stop_fn)
dropdown = DropDown(label="Selection", populate_fn=get_options, on_selection_fn=handle_choice)
```

### Combination Elements

The extension includes specialized builders for complex UI patterns that combine multiple elements:

- {func}`combo_cb_str_builder <isaacsim.gui.components.combo_cb_str_builder>`: Checkbox paired with string field
- {func}`combo_floatfield_slider_builder <isaacsim.gui.components.combo_floatfield_slider_builder>`: Float field with synchronized slider
- {func}`combo_cb_plot_builder <isaacsim.gui.components.combo_cb_plot_builder>`: Checkbox-controlled plot widget
- {func}`combo_cb_xyz_plot_builder <isaacsim.gui.components.combo_cb_xyz_plot_builder>`: Multi-axis plotting with toggle controls

## Key Components

### Data Visualization

**{class}`XYPlot <isaacsim.gui.components.XYPlot>`** provides interactive plotting capabilities with multiple dataset support, automatic axis scaling, and legends. The widget supports hover tooltips showing coordinates and handles data interpolation for smooth visualization.

**{func}`plot_builder <isaacsim.gui.components.plot_builder>`** functions create various plot types including single plots, XYZ coordinate plots, and checkbox-controlled dynamic plots for real-time data display.

### Input Controls

**{class}`StateButton <isaacsim.gui.components.StateButton>`** implements two-state toggle behavior with different text displays and callbacks for each state. It supports physics callback functions that execute continuously while in the active state.

**{class}`DropDown <isaacsim.gui.components.DropDown>`** offers dynamic population through user-defined functions and includes specialized methods for finding USD objects by type, making it particularly useful for scene object selection.

### Layout Elements

**{class}`CollapsableFrame <isaacsim.gui.components.CollapsableFrame>`** and **{class}`ScrollingFrame <isaacsim.gui.components.ScrollingFrame>`** provide structured content organization with expand/collapse functionality and scrollable content areas respectively.

**{func}`setup_ui_headers <isaacsim.gui.components.setup_ui_headers>`** creates standardized extension headers with title, documentation links, overview text, and utility buttons for accessing source code and folders.

### File System Integration

String field builders and wrappers include folder picker integration through `**omni.kit.window.filepicker**`, enabling users to select files and directories with filtering capabilities and bookmark support.

## Integration

The extension uses `**omni.kit.menu.utils**` for creating context menus and accessing standard menu functionality within UI components. Physics integration occurs through `**omni.physics**` dependencies, enabling {class}`StateButton <isaacsim.gui.components.StateButton>` widgets to register physics step callbacks for continuous execution during active states.
