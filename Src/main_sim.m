% 2026 Spring AE50001 / Team5
% Main simulation file

%% MATLAB Initialization
clc; clear; close all;
set(0,'DefaultFigureWindowStyle','docked');

%% Parameters
% Reference: "Nonlinear Model Predictive Control ... ", p.6

% Aircraft Parameters
UAM.m = 2240.73;                        % Mass of the body [kg]
UAM.Ix = 12000;                         % Moment of inertia about X-axis [kg m^2]
UAM.Iy = 9400;                          % Moment of inertia about Y-axis [kg m^2]
UAM.Iz = 20000;                         % Moment of inertia about Z-axis [kg m^2]
               
UAM.Lx = 2.65;                          % Arm length along X-axis [m]
UAM.Lyi = 2.65;                         % Inner motor arm length along Y-axis [m]
UAM.Lyo = 5.5;                          % Outer motor arm length along Y-axis [m]

% Environment Paramters            
Env.g = 9.81;                           % Gravitational acceleration [m/s^2]
Env.rho = 1.225;                        % Air density [kg/m^3]
               
% Propulsive Paramters
Prop.T_max = UAM.m * 2 / 6 * Env.g;     % Maximum thrust [N]
Prop.T_min = 0;                         % Minimum thrust [N]
Prop.cT = 0.02;                         % Moment to thrust ratio

% Control Effectiveness Matrix
UAM.B = [    -1       -1       -1       -1       -1       -1;
         -UAM.Lyi  UAM.Lyi -UAM.Lyo  UAM.Lyo -UAM.Lyi  UAM.Lyi;
          UAM.Lx   UAM.Lx       0        0   -UAM.Lx  -UAM.Lx;
          Prop.cT -Prop.cT -Prop.cT  Prop.cT  Prop.cT -Prop.cT];

% Loss of Effectivenss
LOE = [0 0 0 0 0 0];

%% Simulation Setting
% Simulation Time
tf = 30;
dt = 0.002;
num_step = tf / dt;
t_vec    = (1:num_step) * dt;

% Initial States
X0.uvw = [0; 0; 0];
X0.pqr = [0; 0; 0];
X0.att = [0; 0; 0];
X0.pos = [0; 0; 0];
X0.vel = [0; 0; 0];

X0 = [X0.uvw; X0.pqr; X0.att; X0.pos; X0.vel];  % Initial states
T0 = zeros(6,1);                                % Initial thrust

% Index
IDX.u = 1; IDX.v = 2; IDX.w = 3;
IDX.p = 4; IDX.q = 5; IDX.r = 6;
IDX.phi = 7; IDX.theta = 8; IDX.psi = 9;
IDX.X = 10; IDX.Y = 11; IDX.Z = 12;
IDX.vx = 13; IDX.vy = 14; IDX.vz = 15;

% History record
outsim.X = zeros(15, num_step);
outsim.Ctrl_cmd = zeros(4, num_step);
outsim.Rate_cmd = zeros(3, num_step);
outsim.V_cmd = zeros(3, num_step);
outsim.Thrust = zeros(6, num_step);
outsim.Thrust_cmd = zeros(6, num_step);

%% Control Gain




%% Main Loop






%% Figures







