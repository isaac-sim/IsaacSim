The Kawasaki RS robot URDF files have all been modified such that the +X axis lies in front of the robot to fit with Isaac Sim convention.  This corresponds to the same change in each URDF:

<joint name="world2base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin rpy="0 0 -1.5707963267948966" xyz="0 0 0"/>
</joint>

from the original 

<joint name="world2base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
</joint>.

The URDFs have also been modified to include a new frame in the center of the robot gripper called "gripper_center".  The following has been added at the bottom of each URDF:

<link name="gripper_center"/>
  <joint name="gripper_center_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.0 0.0 .2"/>
    <parent link="onrobot_rg2_base_link"/>
    <child link="gripper_center"/>
</joint>


These modified URDF files were used to generate the Kawasaki RS USD files that are stored on the Nucleus Isaac Sim server.  