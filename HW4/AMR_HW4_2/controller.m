function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
kvz = 70;
kpz = 10;

u1 = params.mass * (params.gravity + des_state.acc(2) + ...
    kvz * (des_state.vel(2) - state.vel(2)) + ...
    kpz * (des_state.pos(2) - state.pos(2)));

kvy = 60;
kpy = 15;

phic = -1 / params.gravity * (des_state.acc(1) + ...
    kvy * (des_state.vel(1) - state.vel(1)) + ...
    kpy * (des_state.pos(1) - state.pos(1)));

phicDot = -1 / params.gravity * (0 + ...
    kvy * 0 + ...
    kpy * (des_state.vel(1) - state.vel(1)));

kvphi = 250;
kpphi = 800;

u2 = params.Ixx * (0 + kvphi * (phicDot - state.omega(1)) ...
    + kpphi * (phic - state.rot(1)));

end

