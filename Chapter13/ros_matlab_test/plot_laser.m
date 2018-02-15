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
function plot_laser( )
    global laser_msg;
    %ROS_MASTER_URI
    ros_master_ip = 'http://192.168.1.6:11311';
    %ROS_HOSTNAME
    matlab_ip = '192.168.1.11';
    %Connect to an external ROS Network, setting ROS_MASTER_URI and
    %ROS_HOSTNAME
    rosinit(ros_master_ip, 'NodeHost', matlab_ip);
    pause(2); % wait a bit the roscore initialization
    
    laser_sub = rossubscriber( '/scan', @get_laser );
    r = rosrate(2); % 2 Hz loop rate 
    for i=1:50
        plot(laser_msg, 'MaximumRange', 7   ); %Plot laser_msg 
        waitfor(r);
    end
    %Shutdown ROS connection
    rosshutdown
    close all
end
