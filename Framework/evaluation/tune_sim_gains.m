function gains = tune_sim_gains(UAM, gains_ref, model_ref, options)
% TUNE_SIM_GAINS  Scale simulation gains from a baseline reference plant.
%
%   Keeps approximate closed-loop bandwidth similar across design changes by
%   scaling rate-loop gains with inertia and vertical-loop gains with mass.

if nargin < 4
    options = struct();
end
if ~isfield(options, 'ratio_min')
    options.ratio_min = 0.5;
end
if ~isfield(options, 'ratio_max')
    options.ratio_max = 2.0;
end

clamp = @(x) min(max(x, options.ratio_min), options.ratio_max);

Ixx_ratio = clamp(UAM.Ix / model_ref.Ix);
Iyy_ratio = clamp(UAM.Iy / model_ref.Iy);
Izz_ratio = clamp(UAM.Iz / model_ref.Iz);
m_ratio   = clamp(UAM.m  / model_ref.m);

gains = gains_ref;
gains.Kp_p  = gains_ref.Kp_p  * Ixx_ratio;
gains.Ki_p  = gains_ref.Ki_p  * Ixx_ratio;
gains.Kp_q  = gains_ref.Kp_q  * Iyy_ratio;
gains.Ki_q  = gains_ref.Ki_q  * Iyy_ratio;
gains.Kp_r  = gains_ref.Kp_r  * Izz_ratio;
gains.Ki_r  = gains_ref.Ki_r  * Izz_ratio;
gains.Kp_vz = gains_ref.Kp_vz * m_ratio;
gains.Ki_vz = gains_ref.Ki_vz * m_ratio;
end
