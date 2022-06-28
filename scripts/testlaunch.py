import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch_path="/home/leus/catkin_ws/src/rope_recog/launch/play_rosbag.launch"

launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path, "bag_file:=fuga"])
launch.start()
launch.shutdown()
