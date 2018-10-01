source /opt/ros/kinetic/setup.zsh

export ROS_NODE_PORT=`get_free_port.py`
export ROS_MASTER_URI=http://asheesh-Aspire-E5-575G:11311/

lf="ros.log"
if [ -e $lf ]; then
  \rm $lf
fi
touch $lf

echo "Starting roscore with port = $ROS_NODE_PORT..."
( ( (stdbuf -oL roscore -p $ROS_NODE_PORT) 1> >(stdbuf -oL sed 's/^/ROSCORE: /') 2>&1 ) >> $lf ) &

cd catkin_ws
catkin_make
source devel/setup.zsh
rosrun two_int_talker two_int_talker.py &
cd ..
