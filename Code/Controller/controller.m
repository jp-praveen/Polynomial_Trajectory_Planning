function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% Getting the constants
g = params.gravity;
m = params.mass;
I = params.I;

% Getting the current and target state data
x_current = state.pos(1);
y_current = state.pos(2);
z_current = state.pos(3);

x_tar = des_state.pos(1);
y_tar = des_state.pos(2);
z_tar = des_state.pos(3);

xdot_current = state.vel(1);
ydot_current = state.vel(2);
zdot_current = state.vel(3);

xdot_tar = des_state.vel(1);
ydot_tar = des_state.vel(2);
zdot_tar = des_state.vel(3);

xddot_tar = des_state.acc(1);
yddot_tar = des_state.acc(2);
zddot_tar = des_state.acc(3);

phi_current = state.rot(1);
theta_current = state.rot(2);
psi_current = state.rot(3);

psi_tar = des_state.yaw;
psidot_tar = des_state.yawdot;

p_current = state.omega(1);
q_current = state.omega(2);
r_current = state.omega(3);

p_des = 0;
q_des = 0;
r_des = psidot_tar;

% Gain tuning
kd1 = 0.1*5;
kd2 = 0.1*5;
kd3 = 0.1*5;
kp1 = 5/1.1;
kp2 = 5/1.1;
kp3 = 5/1.1;

kdphi = .5;
kdtheta = .5;
kdpsi = .5;
kpphi = 50*1.5;
kptheta = 50*2;
kppsi = 20;

% Desired states
xddot_des = xddot_tar + kd1*(xdot_tar-xdot_current) + kp1*(x_tar-x_current);
yddot_des = yddot_tar + kd2*(ydot_tar-ydot_current) + kp2*(y_tar-y_current);
zddot_des = zddot_tar + kd3*(zdot_tar-zdot_current) + kp3*(z_tar-z_current);

phi_des = 1/g*(xddot_des*sin(psi_tar) - yddot_des*cos(psi_tar)); 
theta_des = 1/g*(xddot_des*cos(psi_tar) + yddot_des*sin(psi_tar)); 
psi_des = psi_tar;

% Controls
u1 = m*g + m*zddot_des;

u2 = [kpphi*(phi_des - phi_current) + kdphi*(p_des - p_current);
      kptheta*(theta_des - theta_current) + kdtheta*(q_des - q_current);
      kppsi*(psi_des - psi_current) + kdpsi*(r_des - r_current)];

% Force
F = u1;
% Moment
M = I*u2;


end
