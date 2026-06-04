# Loading Extension
To enable this extension, run Isaac Sim with the flags --ext-folder {path_to_ext_folder} --enable {ext_directory_name}
The user will see the extension appear on the toolbar on startup with the title they specified in the Extension Generator.

This extension may also be enabled through the Extension Manager by providing its local path and searching for it in
"Third Party Extensions"


# Extension Usage
This template extension creates a Load, Reset, and Run button in a simple UI.
The Load and Reset buttons interact with the isaacsim.core.api World() in order
to simplify user interaction with the simulator and provide certain gurantees to the user
at the times their callback functions are called.  

The Run button runs an analogue of a script, which allows the user to run long-lasting
code that takes more than one frame to complete such as moving a robot to a target.
This approach is limited to code that is able to throw a yield() on every frame.


# Template Code Overview
The template is well documented and is meant to be self-explanatory to the user should they
start reading the provided python files.  A short overview is also provided here:

global_variables.py: 
    A script that stores in global variables that the user specified when creating this extension such as the Title and Description.

extension.py:
    A class containing the standard boilerplate necessary to have the user extension show up on the Toolbar.  This
    class is meant to fulfill most ues-cases without modification.
    In extension.py, useful standard callback functions are created that the user may complete in ui_builder.py.

ui_builder.py:
    This file is nearly identical to the ui_builder.py found in the Loaded Scenario template, but the example code in that template is much simpler.
    If your goal is to understand and program a UI that connects to the core.World, you may find it a useful reference.

    This file takes care of building the UI and programming the UI buttons. Here, the user can see useful callback functions that have been
    set up for them, and they may also create UI buttons that are hooked up to more user-defined callback functions.  This file is
    thoroughly documented, and the user should read through it before making serious modification.

scenario.py:
    This file handles the logical flow of this template, including implementing a script to pick up and move a block around obstacles.
    This file demonstrates how to write scripted behavior through the Extension Workflow with minimal overhead or setup.  This is also thoroughly
    documented, and it will be the most interesting read in this template.