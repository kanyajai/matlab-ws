% Helper script that resets odometry
odomResPub = rospublisher('/mobile_base/commands/reset_odometry');
odomResetMsg = rosmessage('std_msgs/Empty');
send(odomResPub, odomResetMsg)