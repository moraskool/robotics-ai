function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
% FILL IN YOUR CODE HERE
Kp = 320;
Kv = 35;
e = s_des(1) - s(1);
e_dot = s_des(2) - s(2);
u = params.mass * ( 0 + Kp*e + Kv*e_dot + params.gravity);
end

