function X_dot = eom_hex(X, T_vec, UAM, Env)
% EOM_HEX  6-DOF rigid-body equations of motion for the hexacopter UAM.
%
%   STATE VECTOR  (15×1)
%   --------------------
%   X(1:3)   = [u; v; w]           body-frame translational velocity [m/s]
%   X(4:6)   = [p; q; r]           body-frame angular velocity [rad/s]
%   X(7:9)   = [phi; theta; psi]   ZYX Euler angles [rad]
%   X(10:12) = [x; y; z]           NED position [m]  (z positive down)
%   X(13:15) = [vx; vy; vz]        NED inertial velocity [m/s]
%                                  (recomputed externally after each step)
%
%   CONVENTION
%   ----------
%   NED frame: x-forward, y-right, z-down.
%   Thrust opposes z-down gravity, so propulsive Fz < 0 (upward).
%   Euler angles use ZYX (yaw-pitch-roll) convention.
%
%   Inputs:
%     X      - state vector [15×1]
%     T_vec  - effective motor thrust vector [6×1], each entry ≥ 0 [N]
%     UAM    - vehicle struct: m, Ix, Iy, Iz, B
%     Env    - environment struct: g
%
%   Output:
%     X_dot  - time derivative of state [15×1]
%              (NED velocity derivatives X_dot(13:15) = zeros, caller updates)

% Unpack state
u = X(1);  v = X(2);  w = X(3);
p = X(4);  q = X(5);  r = X(6);
phi = X(7);  theta = X(8);  psi = X(9);

% Virtual controls from thrust vector:  v_ctrl = B * T  → [Fz; L; M; N]
vc      = UAM.B * T_vec;
Fz_prop = vc(1);       % propulsive force along body z [N], should be < 0
tau_LMN = vc(2:4);     % [L; M; N] body moments [Nm]

% Gravity resolved in body frame (NED, z-down)
gx = -Env.g * sin(theta);
gy =  Env.g * cos(theta) * sin(phi);
gz =  Env.g * cos(theta) * cos(phi);

% Translational dynamics (Newton, body frame, no aerodynamic drag)
u_dot = r*v - q*w + gx;
v_dot = p*w - r*u + gy;
w_dot = q*u - p*v + gz + Fz_prop / UAM.m;

% Rotational dynamics (Euler's equations)
p_dot = (tau_LMN(1) + (UAM.Iy - UAM.Iz)*q*r) / UAM.Ix;
q_dot = (tau_LMN(2) + (UAM.Iz - UAM.Ix)*p*r) / UAM.Iy;
r_dot = (tau_LMN(3) + (UAM.Ix - UAM.Iy)*p*q) / UAM.Iz;

% Euler-angle kinematics (ZYX, singularity-free for |theta| < 90 deg)
phi_dot   = p + (q*sin(phi) + r*cos(phi)) * tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot   = (q*sin(phi) + r*cos(phi)) / cos(theta);

% Position kinematics in NED: dx/dt = R_nb * v_body
sphi = sin(phi);  cphi = cos(phi);
sth  = sin(theta); cth  = cos(theta);
spsi = sin(psi);   cpsi = cos(psi);
R_nb = [cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
        spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
       -sth,        cth*sphi,                    cth*cphi              ];
pos_dot = R_nb * [u; v; w];

% Assemble derivative (NED velocity updated externally, set to zero here)
X_dot = [u_dot; v_dot; w_dot;
         p_dot; q_dot; r_dot;
         phi_dot; theta_dot; psi_dot;
         pos_dot;
         zeros(3,1)];
end
