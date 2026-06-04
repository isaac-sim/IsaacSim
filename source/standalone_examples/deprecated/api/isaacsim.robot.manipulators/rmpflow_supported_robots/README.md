This standalone example provides a generic script for running a follow-target example on any supported robot that uses RMPflow to reach a target while avoiding obstacles.  The purpose of this script is to show only how to use RMPflow, and for the sake of simplicity, it does not use the task/controller paradigm that is typical in other Isaac Sim examples.

The ./supported_robot_follow_target_example.py script takes in runtime with the path to the robot USD asset (which is assumed to be stored on the Nucleus Server) and the name of the robot.  By running the script with the default command line arguments, the list of supported robot names will be printed in the terminal.

###### Command Line Arguments

The supported command-line arguments are as follows:

    -v,--verbose: If True, prints out useful runtime information such as the list of supported robot names that map to RMPflow config files.  Defaults to 'True'

    --robot-name: Name of robot that maps to the stored RMPflow config.  Defaults to "Cobotta_Pro_900"

    --usd-path: Path to robot USD asset on the Nucleus Server.  Defaults to "/Isaac/Robots/Denso/cobotta_pro_900.usd".  The typical location of a specific robot is under 
                "/Isaac/Robots/{manufacturer_name}/{robot_name}/{robot_name}.usd"

    --add-orientation-target: Add the orientation of the target cube to the RMPflow target. Defaults to False.


###### Choosing Correct Robot Name Argument

With the default arguments, the above script will run using the Cobotta Pro 900 robot and will produce the following output:

    Names of supported robots with provided RMPflow config
        ['Franka', 'UR3', 'UR3e', 'UR5', 'UR5e', 'UR10', 'UR10e', 'UR16e', 'Rizon4', 'Cobotta_Pro_900', 'Cobotta_Pro_1300', 'RS007L', 'RS007N', 'RS013N', 'RS025N', 'RS080N', 'FestoCobot']

    Successfully referenced RMPflow config for Cobotta_Pro_900.  Using the following parameters to initialize RmpFlow class:
    {
        'end_effector_frame_name': 'gripper_center',
        'ignore_robot_state_updates': False,
        'maximum_substep_size': 0.00334,
        'rmpflow_config_path': '/path/to/omni_isaac_sim/_build/linux-x86_64/release/extsDeprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/./Denso/cobotta_pro_900/rmpflow/cobotta_rmpflow_common.yaml',
        'robot_description_path': '/path/to/omni_isaac_sim/_build/linux-x86_64/release/extsDeprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/./Denso/cobotta_pro_900/rmpflow/robot_descriptor.yaml',
        'urdf_path': '/path/to/omni_isaac_sim/_build/linux-x86_64/release/extsDeprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/./Denso/cobotta_pro_900/rmpflow/../cobotta_pro_900_gripper_frame.urdf'
    }

The names of supported robots are suitable for the `--robot-name` argument, and each must correctly correspond to the robot USD path.  In a future release, configuration data for supported robots will be centralized such that only a single argument will be required.  The specific method of accessing supported robot RMPflow configs provided here will then be deprecated.

The remaining output shows the RMPflow configuration information that is found under the name "Cobotta_Pro_900".  This configuration is used to initialize the `RmpFlow` class.


###### Examples of loading other robots

Multiple valid combinations of command line arguments are shown for different supported robots:

    python.sh supported_robot_follow_target_example.py --robot-name RS080N --usd-path "/Isaac/Robots/Kawasaki/RS080N/rs080n_onrobot_rg2.usd"

    python.sh supported_robot_follow_target_example.py --robot-name UR16e --usd-path "/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd"

    python.sh supported_robot_follow_target_example.py --robot-name FestoCobot --usd-path "/Isaac/Robots/Festo/FestoCobot/festo_cobot.usd"


