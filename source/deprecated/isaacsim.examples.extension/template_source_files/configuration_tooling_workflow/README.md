# Loading Extension
To enable this extension, run Isaac Sim with the flags --ext-folder {path_to_ext_folder} --enable {ext_directory_name}
The user will see the extension appear on the toolbar on startup with the title they specified in the Extension Generator

# Extension Usage
This template extension creates a basic tool for interacting with a robot Articulation
through the UI by changing its joint position targets.  This format is generally useful
for building UI tools to help with robot configuration or inspection.

To use the template as written, the user must load a robot Articulation onto the stage,
and press the PLAY button on the left-hand toolbar.  Then, in the extension UI, they can select their
Articulation from a drop-down menu and start controlling the robot joint position targets.

The extension only functions while the timeline is running because robot Articulation objects only
function while the timeline is playing (physics does not run while the timeline is stopped stopped).


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