%% This function subscribe to a topic containing laser scanner data plotting them
%  
%%
 % Copyright (C) 2017, Jonathan Cacace.
 % Email id : jonathan.cacace@gmail.com
 % Redistribution and use in source and binary forms, with or without
 % modification, are permitted provided that the following conditions are met:
 %   * Redistributions of source code must retain the above copyright notice,
 %     this list of conditions and the following disclaimer.
 %   * Redistributions in binary form must reproduce the above copyright
 %     notice, this list of conditions and the following disclaimer in the
 %     documentation and/or other materials provided with the distribution.
 %   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 %     contributors may be used to endorse or promote products derived from
 %     this software without specific prior written permission.
 %
 % THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 % AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 % IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 % ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 % LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 % CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 % SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 % INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 % CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 % ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 % POSSIBILITY OF SUCH DAMAGE.
 %
%%
function vfh_obstacle_avoidance(ros_master_ip, matlab_ip )
      
    %Init roscore
    rosinit(ros_master_ip, 'NodeHost', matlab_ip);
    pause(2) % wait a bit the roscore initialization
    
    %Subscribe to laser scanner data
    laserSub = rossubscriber('/scan');
    odomSub =  rossubscriber('/odom');
    %Advertise robot velocity
    [velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
    
    %Initialize Vector Field Histogram obstacle avoidance and its gains
    vfh = robotics.VectorFieldHistogram;
    vfh.DistanceLimits = [0.05 1];
    vfh.RobotRadius = 0.1;
    vfh.MinTurningRadius = 0.2;
    vfh.SafetyDistance = 0.1;

    r_max = 6.28;
    r_min = 0.0;
    ob_dist = [];
    omod_vel = [];
    
    rate = robotics.Rate(10);
    
    odom_vel_x = [];
    odom_vel_z = [];
    
    odom_pos_x = [];
    odom_pos_y = [];
    while rate.TotalElapsedTime < 25

         % Get laser scan data
         laserScan = receive(laserSub);
         odom = receive(odomSub);
         
         ranges = double(laserScan.Ranges);
         angles = double(laserScan.readScanAngles);
 
         odom_vel_x = [odom_vel_x, odom.Twist.Twist.Linear.X];
         odom_vel_z = [odom_vel_z, odom.Twist.Twist.Angular.Z];
         odom_pos_x = [odom_pos_x, odom.Pose.Pose.Position.X];
         odom_pos_y = [odom_pos_y, odom.Pose.Pose.Position.Y];
         targetDir = (r_max-r_min).*rand();
         % Call VFH object to computer steering direction
          steerDir= vfh(ranges, angles, targetDir);
         ob_dist = [ob_dist, min(ranges) ];
         % Calculate velocities
         if ~isnan(steerDir) % If steering direction is valid
             desiredV = 0.2;
             w = exampleHelperComputeAngularVelocity(steerDir, 1);
         else % Stop and search for valid direction
             desiredV = 0.0;
             w = 0.5;
         end
 
         % Assign and send velocity commands
         velMsg.Linear.X = desiredV;
         velMsg.Angular.Z = w;
         velPub.send(velMsg);
         
         waitfor(rate);
     end
    
    rosshutdown
    
    figure(1);
    plot( ob_dist, 'red-' );
    legend('obstacle distance');
    ylabel( 'm' );
    grid on;
    title('obstacle distance');
    
    figure(2);
    plot( odom_vel_x, 'red' );
    legend('Forward velocity');
    ylabel( 'm/s' );
    grid on
    title('forward velocity');
    
    figure(3);
    plot( odom_vel_z, 'blue' );
    legend('Angular velocity');
    ylabel( 'rad/s' );
    grid on
    title('angular velocity');
    
    figure(4)
    plot( odom_pos_x, odom_pos_y, 'red');
    xlabel('x');
    ylabel('y');
    title('path');
    grid on
end
