# Changelog
## [1.0.12] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.0.11)

## [1.0.11] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.0.10] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.0.9] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.0.8] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.7] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.0.6] - 2025-04-22
### Changed
- Update to new Surface Gripper

## [1.0.5] - 2025-04-09
### Changed
- Update Isaac Sim robot asset path

## [1.0.4] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.3] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-09-28
### Changed
-  Extension renamed to isaacsim.cortex.framework

## [0.3.9] - 2024-07-25
### Removed
- Deprecation tag

## [0.3.8] - 2024-05-22
### Added
- Deprecation tag

## [0.3.7] - 2024-03-06
### Changed
- Updated path to ur10

## [0.3.6] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [0.3.5] - 2023-10-10
### Fixed
- Conversion from Rotation matrix Quaternion uses direct method

## [0.3.4] - 2023-06-12
### Fixed
- omni.isaac.kit dependency

## [0.3.3] - 2023-03-23
### Added
- __str__ methods for Df and Dfb classes

## [0.3.2] - 2023-03-13
### Fixed
- `CortexGripper.step()` bugfix: change `self.command = None` to `self.clear()` since `self.command` is a property.

## [0.3.1] - 2023-02-21
### Fixed
- Fix examples DfContext -> {DfBasicContext, DfRobotApiContext}

## [0.3.0] - 2023-02-12
### Fixed
- Cleanup pass through isaacsim.cortex.framework
    - Comments, type hints, and some cleanup
    - Remove unused `cortex_task.py` file
    - Remove the `df_behavior_watcher.py` (no longer used)
    - Convert all monitor adds in behavior to use the built in `add_monitors()`

## [0.2.11] - 2023-01-03
### Changed
- `isaacsim.cortex.framework` module is exposed on startup

## [0.2.10] - 2022-12-12
### Added
- `MotionCommander.soft_reset()` method to reset only the C-space integration state.

## [0.2.9] - 2022-12-09
### Fixed
- Make `peck_decider_network.py` example align better with its `peck_state_machine.py` counterpart

## [0.2.8] - 2022-12-09
### Fixed
- During cortex reset, reset commanders before behaviors. (Motion commander reset adds obstacles back; behavior might disable them.)

## [0.2.7] - 2022-12-08
### Fixed
- Fix motion commander reset bug: reset the target prim to the end-effector
- Add missing suction gripper reset in CortexUr10

## [0.2.6] - 2022-12-08
### Changed
- Expose the cortex pipeline reset separate from the full world reset. This is used by `cortex_ros` now when synchronizing with a sim/physical robot.

## [0.2.5] - 2022-12-07
### Fixed
- Robustness of block stacking with and without ROS sync
- Block stacking:
    - fix: hang when block too close to base, too close to another block, or too far away (better grasp choices)
    - fix: lift to a consistent height rather than based on timing
    - fix: lift cautiously when near another block
- Fix: ROS sync with block stacking -- hang trying to pick up block after dropping it. (Wasn't triggering the `is_open` clause because the fingers weren't opening far enough, possibly because of friction. Made that setting less stringent.)
- Some minor trimming including removing redundant obstacle monitors from UR10 bin stacking and unused monitors from franka block stacking.

## [0.2.4] - 2022-12-04
### Changed
- Updates to standalone examples as we prepare tutorials
    - remove leonardo; use standard block stacking franka example (`simple_franka_examples` --> `franka_examples`)
    - simplify follow example

## [0.2.3] - 2022-12-01
### Changed
- Bugfixes and refactoring in UR10 example. Includes introducing obstacle monitors, adding corrective reactions during placement, smoothing out the bin flip behavior, and some misc bugfixes.
- Modifications to CortexWorld API for adding decider networks to support a dirt-simple follow example with tutorial modifications.
- `loop_fast` param in cortex world `run()` method and rename `step_loop_runner()` to `run()`.
- Change `DfNetwork` `decider` parameter to `root` (more explicit).

## [0.2.2] - 2022-11-23
### Changed
- commander API framework with explicit logical state monitoring, behavior stepping, and commander stepping.
- suite of examples / demos in `standalone_examples/api/isaacsim.cortex.framework`. Includes Cortex versions of the Leonardo demo and UR10 bin stacking.

## [0.2.1] - 2022-10-28
### Changed
- Bugfix: isaacsim.cortex.framework extension shouldn't depend on ROS.
- Improve the cortex task API so it takes the commander rather than just the target prim.

## [0.2.0] - 2022-10-24
### Changed
- Moved ROS dependent tools and extensions to `omni.isaac.cortex_sync`.
- Add cortex task tools for native use of decider networks and the motion commander from the core api.

## [0.1.13] - 2022-08-31
### Changed
- Update paths to 2022.2

## [0.1.12] - 2022-08-03
### Fixed
- Switch to new gripper API
- Bugfix: behavior modules don't load if the file doesn't exist on startup.

## [0.1.11] - 2022-06-01
### Fixed

- prevent cortex launch from ROS-sourced env (inc. removing spamming prints from cortex_sim when launched into belief-only mode with ROS enabled)

## [0.1.10] - 2022-05-27
### Changed
- update the readme files
- remove <close_gripper> printout
- ensure obstacles enabled when building block stacking behavior

## [0.1.9] - 2022-05-26
### Changed
- Example behaviors now set their control mode (position-only or full pose commands) on construction. Prevents user error.
- Bugfix: go_home was passing in full c-space config for posture config preventing other scripts from passing only active joints. Expose active joints through motion commander and correct error.
- Add franka/nullspace.py demo of nullspace behavior using the `posture_config` motion command parameter.

## [0.1.8] - 2022-05-26
### Changed

- Cleanup tests to make them presentable. Keeps only `test_motion_commander.py` and `test_df.py`
- Switch `cortex_main.py` to have `--usd_env` be relative and add `--assets_root` flag which will default to using the built in tool to find the standard assets path.
- Add some standalone examples referenced by the tutorials. Some "simple examples" showing basic concepts, and some small but more complete demos showing programming paradigms.
- Some utilities used by examples: make_rotation_matrix(), tick_action()
- Automatically add monitors from context object (prevent users from forgetting; makes examples more concise); backward compatible.
- bugfix: motion commander wasn't handling position-only targets correctly.
- bugfix: cortex_ros was suppressing gripper pubs

## [0.1.7] - 2022-05-23
### Changed

- Bugfix [OM-51613] Cortex script help issues
- Bugfix [OM-51762] Fix transient error in cortex_ros when ROS messaging is still initializing
- Bugfix in SmoothedCommand: wasn't projecting onto a valid rotation after blending matrices.
- Include spliting out proj_R out from proj_T in math_util.py
- Remove unused alpha_diminish member from SmoothedCommand
- Comment code

## [0.1.6] - 2022-05-18
### Changed

- Switch from lula_ros to cortex_control (renamed internal library to match the ros_workspace deployment).

## [0.1.5] - 2022-05-17
### Changed

- Replaced find_nucleus_server() with get_assets_root_path()

## [0.1.4] - 2022-05-16
### Changed

- convert to default meters
- bugfixes around setting up world as singleton and accessing in extensions, including creating robot objects from extensions.
- includes a hack to handle world.reset bug where gains reset as well.
- add comments to tools.py and math.py
- updated READMEs to point to .../Isaac/Samples/Cortex/...
- fix ur10 default config setting
- generalize hiding of object property prims

## [0.1.3] - 2022-05-06
### Changed

- converted all imports to full paths `import isaacsim.cortex.framework.<component>`.
    - works uniformly between extensions and python app (loop runner); no need to augment the python path
    - fixes a bug where the `df_behavior_module.py` change was noticed by `df_behavior_watcher.py`, but it couldn't be loaded.
- updates READMEs (proofread and added some description).

## [0.1.2] - 2022-05-04
### Changed

- Change USD path convention from `/cortex/world/...`  to `/cortex/belief/...` to match the cortex terminology.  Also, there are aspects of the core API that automatically add `/World` so that was overloaded.
- Move behaviors to `standalone_examples/cortex`
    - separate into robot independent and robot specific (franka) scripts
    - fix robot independent behaviors to actually be robot independent, including generalized go_home to work with multiple robots
    - add script for launching cortex, activating behaviors, and clearing the current behavior
    - Update readmes

## [0.1.1] - 2022-05-02
### Changed

- Add a --test flag to cortex_main.py to run a short bringup test. (Bringup, wait 2 secs, shutdown.)
- cortex_main.py loads without ROS now by default. Use --enable_ros to bring up cortex_{ros,sim}.
- Add support for UR10. Includes making USD path convensions generic, adding cortex:robot_type attribute to robot USD, and updating the loading tools and motion policy configs.
- Fix ctrl-c issue: ros was preventing ctrl-c out of cortex_main.py.

## [0.1.0] - 2022-04-27
### Added

- Initial version of Isaac Cortex
