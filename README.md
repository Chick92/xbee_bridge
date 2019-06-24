# rfd900_bridge

ros package intended to allow a ROS bridge of various topics (just cmd vel at the moment) between two machines with independant ROS masters, for situations where either a multi master network is not possible, or some networking issue prevents other solutions using tcp/ip.

This can be implemented with any serial based radio, or wired serial connection. 

package is based on the rfd900_bridge package by wagnera - https://github.com/wagnera/rfd900_bridge
