%% MAIN_MDO_Adv.m (고도화된 MDO 실행 스크립트)
clc; clear; close all;

fprintf('===============================================================\n');
fprintf(' Advanced UAM MDO: Regulatory Integration & Weight Sensitivity\n');
fprintf('===============================================================\n\n');

% 초기화
cfg = mdo_config();
d_base = design_default();

%% Section 1: Weight Sensitivity Analysis (가중치 튜닝 및 민감도 확인)
fprintf('Section 1: Fault-Tolerance Weight Sensitivity Sweep\n');
% 고장 허용 가중치(f3)를 0.1부터 0.4까지 변화시키며 최적화 수행
fault_weights = 0.1:0.1:0.4;
results_sweep = struct('weight', {}, 'mass', {}, 'T_max', {}, 'J_comb', {});

for i = 1:length(fault_weights)
    % 가중치 동적 변경 (질량 0.3 고정, 고장 가중치 변화)
    w_fault = fault_weights(i);
    w_power = 0.9 - (0.3 + w_fault); % 총합 1.0 유지를 위한 보정 (단순화)
    
    test_cfg = cfg;
    test_cfg.objectives.stage1.weights = [0.3, w_power, w_fault, 0.1, 0.0];
    
    fprintf('  Running SOO with Fault Weight: %.1f...\n', w_fault);
    [d_opt, J_opt, ~] = run_soo(d_base, test_cfg);
    
    % 최적화 결과 저장
    results_sweep(i).weight = w_fault;
    results_sweep(i).mass = eval_design(d_opt, UAMOptions(test_cfg, 'Mode', 'acs')).acs.mass;
    results_sweep(i).T_max = d_opt.T_max;
    results_sweep(i).J_comb = J_opt;
end

% 가중치에 따른 질량 변화 플롯 출력
figure('Name', 'Sensitivity: Weight vs Mass');
plot([results_sweep.weight], [results_sweep.mass], '-ok', 'LineWidth', 2);
xlabel('Fault Thrust Margin Weight (f3)'); ylabel('Optimized Total Mass [kg]');
title('Cost of Safety: Mass Increase vs Fault Weight');
grid on;

%% Section 2: Cost of Safety Quantitative Comparison (안전 비용 정량 분석)
fprintf('\nSection 2: Cost of Safety (Nominal vs Fault-Tolerant)\n');

% 1. Nominal-Opt (성능 위주 최적화: 고장 가중치 0)
cfg_nom = cfg;
cfg_nom.objectives.stage1.weights = [0.5, 0.5, 0.0, 0.0, 0.0];
[d_nom, J_nom, ~] = run_soo(d_base, cfg_nom);

% 2. Fault-Opt (고장 허용 최적화: 고장 가중치 0.4)
cfg_fault = cfg;
cfg_fault.objectives.stage1.weights = [0.2, 0.2, 0.4, 0.2, 0.0];
[d_fault, J_fault, ~] = run_soo(d_base, cfg_fault);

% 3. 평가 및 정량 비교
res_nom = eval_design(d_nom, UAMOptions(cfg_nom, 'Mode', 'acs'));
res_fault = eval_design(d_fault, UAMOptions(cfg_fault, 'Mode', 'acs'));

mass_penalty = (res_fault.acs.mass - res_nom.acs.mass) / res_nom.acs.mass * 100;
Tmax_increase = (d_fault.T_max - d_nom.T_max) / d_nom.T_max * 100;

fprintf('-------------------------------------------------------\n');
fprintf(' Metric          | Nominal-Opt | Fault-Opt | Difference\n');
fprintf('-------------------------------------------------------\n');
fprintf(' Mass [kg]       | %11.2f | %9.2f | %+5.1f %%\n', res_nom.acs.mass, res_fault.acs.mass, mass_penalty);
fprintf(' T_max [N]       | %11.2f | %9.2f | %+5.1f %%\n', d_nom.T_max, d_fault.T_max, Tmax_increase);
fprintf(' Thrust Margin   | %11.2f | %9.2f |  - \n', res_nom.acs.hover_margin, res_fault.acs.hover_margin);
fprintf('-------------------------------------------------------\n');
fprintf('=> 안전(Fault-Tolerance)을 확보하기 위해 질량은 %.1f%%, 추력 요구량은 %.1f%% 증가함.\n', mass_penalty, Tmax_increase);

%% Section 3: Stage 2 Mission Validation for Fault-Opt
fprintf('\nSection 3: Stage 2 Mission Verification for Fault-Opt\n');

% 8자 비행(Figure-8) 미션 설정
mission_cfg              = cfg.sim;
mission_cfg.scenario     = 'figure8';
mission_cfg.dt           = 0.01;
mission_cfg.t_end        = 140;
mission_cfg.t_fault      = 20;   % 상승 직후인 20초 지점에 고장 주입
mission_cfg.mission.A         = 100;
mission_cfg.mission.T_period  = 120;
mission_cfg.mission.z_cruise  = 50;
mission_cfg.mission.t_start   = 20;
mission_cfg.mission.n_laps    = 1;
mission_cfg.mission.ramp_time = 0;

% 앞에서 구한 고장 허용 설계안(d_fault)을 8자 비행 시뮬레이션에 투입
fprintf('  Running full dynamic simulation for Fault-Opt design...\n');
result_fault_mission = eval_design(d_fault, UAMOptions(cfg_fault, ...
    'Mode',         'full', ...
    'Verbose',      true, ...
    'Fault',        [1;0;0;0;0;0], ... % 1번 모터 완전 고장 모사
    'SimConfig',    mission_cfg, ...
    'ObjectiveSet', 'stage2'));

% 시뮬레이션 결과(그래프 및 3D 비행 궤적) 시각화
visualize_results(result_fault_mission, 'sim');

fprintf('\n===============================================================\n');
fprintf(' Advanced MDO Analysis Complete.\n');
fprintf('===============================================================\n');
