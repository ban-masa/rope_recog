#!/usr/bin/env python
import sys

import roslaunch
print(sys.argv)

parser = roslaunch._get_optparse()
(options, args) = parser.parse_args(sys.argv[1:])
#print(options)
print(args)
args = roslaunch.rlutil.resolve_launch_arguments(args)
print(args)

#roslaunch.main()
