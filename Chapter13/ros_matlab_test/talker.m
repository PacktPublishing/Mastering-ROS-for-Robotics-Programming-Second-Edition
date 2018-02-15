%ROS_MASTER_URI
rosshutdown
ros_master_ip = 'http://192.168.1.6:11311';
%ROS_HOSTNAME
matlab_ip = '192.168.1.11';
%Connect to an external ROS Network, setting ROS_MASTER_URI and
%ROS_HOSTNAME
rosinit(ros_master_ip, 'NodeHost', matlab_ip);
pause(2) % wait a bit the roscore initialization
%Subscribe to the topic: /talker
talker_sub = rossubscriber( '/talker' );
%Advertise a String on topic /chatter
[chatter_pub, chatter_msg] = rospublisher('/chatter','std_msgs/String');
r = rosrate(2); % 2 Hz loop rate
pause(2) % wait a bit the roscore initialization
for i = 1:20
    %Get data from the input topic
    data = talker_sub.LatestMessage;
    chatter_msg.Data = data.Data;
    %Publish data on the output topic
    send(chatter_pub, chatter_msg);
    %Wait for the control loop rate
    waitfor(r);
end
%Shutdown ROS connection
rosshutdown