function d = design_default()
% DESIGN_DEFAULT  Returns the baseline hexacopter design vector.
%
%   The baseline corresponds to the UAM vehicle defined in the reference
%   simulation scripts (sim_control.m, sim_path_following.m).
%
%   Design variable fields
%   ----------------------
%   d.Lx    : fore/aft arm length [m]        (motor rows 1-2 and 5-6)
%   d.Lyi   : inner lateral arm length [m]   (motors 1,2,5,6)
%   d.Lyo   : outer lateral arm length [m]   (motors 3,4 - mid row)
%   d.T_max : maximum thrust per motor [N]
%   d.cT    : moment-to-thrust ratio [-]     (propeller geometric property)
%   d.m     : total vehicle mass [kg]
%
%   Motor layout in body frame (NED, x=forward, y=right, z=down):
%   ---------------------------------------------------------------
%     Motor 1: (+Lx, -Lyi)   front-left    spin: CCW
%     Motor 2: (+Lx, +Lyi)   front-right   spin: CCW
%     Motor 3: (  0, -Lyo)   mid-left      spin: CW
%     Motor 4: (  0, +Lyo)   mid-right     spin: CW
%     Motor 5: (-Lx, -Lyi)   rear-left     spin: CCW
%     Motor 6: (-Lx, +Lyi)   rear-right    spin: CW
%   (Spin signs → yaw row of B: [-cT, -cT, +cT, +cT, -cT, +cT])
%
%   Reference: AE50001 Team5 baseline, 2026.
%
%   VEHICLE MODEL
%   -------------
%   d.use_vehicle_model = true activates physics-based mass via vehicle_model.m
%   (Delbecq 2020 scaling laws).  d.m is an approximate initial value used
%   only when the vehicle model is disabled (cfg.model.use_vehicle_model=false).

d.Lx      = 2.65;    % [m]  fore/aft arm
d.Lyi     = 2.65;    % [m]  inner lateral arm (front/rear)
d.Lyo     = 5.50;    % [m]  outer lateral arm (middle)
d.cT      = 0.03;    % [-]  moment-to-thrust ratio
d.d_prop  = 0.40;    % [m]  reference propeller diameter

% Vehicle model activation: set m_payload to enable physics-based mass
d.m_payload = 1500.0;  % [kg] fixed UAM payload budget
d.use_vehicle_model = true;

% T_max: 2× hover thrust margin at reference total mass (2240.73 kg)
g       = 9.81;
d.T_max = 2240.73 * 2.0 / 6.0 * g;  % ≈ 7327 N per motor

d.m = 2240.73;  % [kg] approximate; vehicle_model overrides when use_vehicle_model=true
end
