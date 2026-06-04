# Loading Extension
To enable this extension, run Isaac Sim with the flags --ext-folder {path_to_ext_folder} --enable {ext_directory_name}
The user will see the extension appear on the toolbar on startup with the title they specified in the Extension Generator


# Extension Usage
This template extension creates a Load, Reset, and Run button in a simple UI.
The Load and Reset buttons interact with the isaacsim.core.api World() in order
to simplify user interaction with the simulator and provide certain gurantees to the user
at the times their callback functions are called.  


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
    This file is the user's main entrypoint into the template.  Here, the user can see useful callback functions that have been
    set up for them, and they may also create UI buttons that are hooked up to more user-defined callback functions.  This file is
    the most thoroughly documented, and the user should read through it before making serious modification.

scenario.py:
    This file contains an implementation of an example "Scenario" that implements a "teardown", "setup", and "update" function.
    This particular structure was chosen to make a clear code separation between UI management and the scenario logic.  In this way, the 
    ExampleScenario() class serves as a simple backend to the UI.  The user should feel encouraged to implement the backend to their UI
    that best suits their needs.
