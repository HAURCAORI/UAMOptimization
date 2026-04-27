function ref = figure8_reference(time, mission)
% FIGURE8_REFERENCE  Reference states for a climb-plus-figure-8 mission.
%
%   The mission starts with a vertical climb, then transitions to a
%   Lissajous 1:2 figure-8 in the horizontal plane. The horizontal mission
%   is blended in smoothly so the vehicle does not jump from hover to a
%   nonzero lateral reference velocity at mission start.

if time < mission.t_start
    climb_ratio = time / max(mission.t_start, 1e-6);

    ref.x = 0;
    ref.y = 0;
    ref.vx = 0;
    ref.vy = 0;
    ref.ax = 0;
    ref.ay = 0;

    ref.alt = mission.z_cruise * climb_ratio;
    ref.vz_up = mission.z_cruise / max(mission.t_start, 1e-6);
else
    t_traj = time - mission.t_start;
    w = mission.omega;
    A = mission.A;
    [blend, blend_dot, blend_ddot] = mission_blend(t_traj, mission);

    x_raw  = A * sin(w * t_traj);
    y_raw  = A * sin(2 * w * t_traj);
    vx_raw = A * w * cos(w * t_traj);
    vy_raw = 2 * A * w * cos(2 * w * t_traj);
    ax_raw = -A * w^2 * sin(w * t_traj);
    ay_raw = -4 * A * w^2 * sin(2 * w * t_traj);

    ref.x  = blend * x_raw;
    ref.y  = blend * y_raw;
    ref.vx = blend_dot * x_raw + blend * vx_raw;
    ref.vy = blend_dot * y_raw + blend * vy_raw;
    ref.ax = blend_ddot * x_raw + 2 * blend_dot * vx_raw + blend * ax_raw;
    ref.ay = blend_ddot * y_raw + 2 * blend_dot * vy_raw + blend * ay_raw;

    ref.alt = mission.z_cruise;
    ref.vz_up = 0;
end
end

function [s, s_dot, s_ddot] = mission_blend(t_traj, mission)
ramp_time = 0;
if isfield(mission, 'ramp_time')
    ramp_time = max(mission.ramp_time, 0);
end

if ramp_time <= 0
    s = 1;
    s_dot = 0;
    s_ddot = 0;
    return;
end

tau = min(max(t_traj / ramp_time, 0), 1);
s = 6 * tau^5 - 15 * tau^4 + 10 * tau^3;
ds_dtau = 30 * tau^4 - 60 * tau^3 + 30 * tau^2;
d2s_dtau2 = 120 * tau^3 - 180 * tau^2 + 60 * tau;

s_dot = ds_dtau / ramp_time;
s_ddot = d2s_dtau2 / (ramp_time^2);
end
