#!/usr/bin/env python

import os
import roslaunch
import tkinter
import tkinter.filedialog
import sys

iDir = os.path.abspath(os.path.dirname(__file__))
iDir = os.path.join(iDir, "../rosbag")
filename = tkinter.filedialog.askopenfilename(initialdir=os.path.join(iDir, "../rosbag"))

print(filename)

#roslaunch.main([sys.argv[0], 'rope_recog', 'play_rosbag.launch', 'bag_file:={}'.format(filename)])

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args1 = ['rope_recog', 'play_rosbag.launch', 'bag_file:={}'.format(filename)]
roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
roslaunch_args1 = cli_args1[2:]

cli_args2 = ['rope_recog', 'rope_recog.launch']
roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)
roslaunch_args2 = cli_args2[2:]

launch_files = [(roslaunch_file1[0], roslaunch_args1), (roslaunch_file2[0], roslaunch_args2)]

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
parent.start()
parent.spin()
