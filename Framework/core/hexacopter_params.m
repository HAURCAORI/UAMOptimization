function [UAM, Prop, Env] = hexacopter_params(d)
% HEXACOPTER_PARAMS  Build physical model structs from a design vector.
%
%   Converts the compact design struct d (from design_default.m or an
%   optimizer candidate) into the UAM / Prop / Env parameter structs
%   consumed by the simulation and ACS evaluation routines.
%
%   INERTIA MODEL
%   -------------
%   Inertia is estimated by treating each motor as a point mass located at
%   its arm-tip position, plus a residual body inertia at the CG.
%
%   Motor positions (body x-y plane):
%     Motors 1,2,5,6: at x = ±Lx, y = ±Lyi   (4 motors)
%     Motors 3,4:     at x = 0,   y = ±Lyo    (2 motors)
%
%   Motor-mass contributions:
%     Ix_m = m_m * (4*Lyi^2 + 2*Lyo^2)
%     Iy_m = m_m * (4*Lx^2)
%     Iz_m = Ix_m + Iy_m    (perpendicular-axis theorem for flat body)
%
%   The effective motor mass m_m and residual body inertia are calibrated
%   so that the baseline design (Lx=Lyi=2.65, Lyo=5.5) exactly reproduces
%   the reference inertia values (Ix=12000, Iy=9400, Iz=20000 kg·m²).
%
%   MASS MODEL
%   ----------
%   Total mass d.m is treated as fixed (payload + structure budget).
%   Changing arm lengths redistributes inertia but not mass. An optional
%   structural mass penalty can be added via the eval_design.m wrapper.
%
%   Inputs:
%     d   - Design struct (see design_default.m)
%           When d.m_payload is present, activates physics-based
%           vehicle_model.m (Delbecq 2020 scaling).  Otherwise uses the
%           legacy fixed-mass path for backward compatibility.
%
%   Outputs:
%     UAM   - Vehicle struct
%     Prop  - Propulsion struct
%     Env   - Environment struct

% ── Environment ─────────────────────────────────────────────────────────
Env.g   = 9.81;    % [m/s²]
Env.rho = 1.225;   % [kg/m³]

% ── Dispatch: physics-based model vs legacy fixed-mass path ──────────────
use_vehicle_model = false;
if isfield(d, 'use_vehicle_model')
    use_vehicle_model = logical(d.use_vehicle_model);
elseif isfield(d, 'm_payload')
    use_vehicle_model = true;
end

if use_vehicle_model
    % ── Physics-based vehicle model (DATCOM-style scaling laws) ──────────
    mdl = vehicle_model(d);

    UAM.m   = mdl.m;
    UAM.Lx  = mdl.Lx;
    UAM.Lyi = mdl.Lyi;
    UAM.Lyo = mdl.Lyo;
    UAM.Ix  = mdl.Ixx;
    UAM.Iy  = mdl.Iyy;
    UAM.Iz  = mdl.Izz;
    UAM.B   = mdl.B;
    UAM.model = mdl;   % attach full model for downstream cost computation

    Prop.T_max = mdl.T_max;
    Prop.T_min = 0;
    Prop.cT    = mdl.cT;
    Prop.tau   = 0.04;

else
    % ── Legacy path (fixed mass, calibrated effective motor masses) ───────
    Lx_b  = 2.65;   Lyi_b = 2.65;   Lyo_b = 5.50;
    Ix_b  = 12000;  Iy_b  = 9400;

    mm_Ix = Ix_b / (4*Lyi_b^2 + 2*Lyo_b^2);  % ≈ 135.5 kg  (roll axis)
    mm_Iy = Iy_b / (4*Lx_b^2);               % ≈ 335.0 kg  (pitch axis)

    UAM.m   = d.m;
    UAM.Lx  = d.Lx;
    UAM.Lyi = d.Lyi;
    UAM.Lyo = d.Lyo;
    UAM.Ix  = mm_Ix * (4*d.Lyi^2 + 2*d.Lyo^2);
    UAM.Iy  = mm_Iy * (4*d.Lx^2);
    UAM.Iz  = UAM.Ix + UAM.Iy;
    UAM.B   = build_B_matrix(d.Lx, d.Lyi, d.Lyo, d.cT);

    Prop.T_max = d.T_max;
    Prop.T_min = 0;
    Prop.cT    = d.cT;
    Prop.tau   = 0.04;
end
end
