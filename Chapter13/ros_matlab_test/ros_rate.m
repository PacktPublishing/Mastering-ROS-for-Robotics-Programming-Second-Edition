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
classdef ros_rate < matlab.System
   
    % Public, tunable properties
    properties
        RATE;
        
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        rateObj;
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            %obj.RateObj = robotics.Rate(1/obj.SampleTime);
            obj.rateObj = robotics.Rate(obj.RATE);
        end

        function stepImpl(obj)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            obj.rateObj.waitfor();

        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
