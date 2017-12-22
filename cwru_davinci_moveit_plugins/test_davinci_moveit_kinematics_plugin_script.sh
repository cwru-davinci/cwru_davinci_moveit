clear

echo "Testing davinci moveit davinci moveit kinematics plugin is going to begin."

cd /home/sulu/ros_ws/src/cwru_davinci_moveit/cwru_davinci_moveit_plugins/cwru_davinci_moveit_kinematics_plugin/test

rostest cwru_davinci_moveit_plugins test_davinci_moveit_kinematics_plugin.test
