%% Init
clear all
close all
clc
addpath(genpath(cd));
load('system/parameters_scenarios.mat');
param = compute_controller_base_parameters;

T_sp = param.T_sp;
T0_1 = T_sp + [3;1;0];
T0_2 = T_sp + [-1;-0.1;-4.5];

%% clear persisten variables
clear controller_lqr; 
clear controller_mpc_1; 
clear controller_mpc_2; 
clear controller_mpc_3; 
clear controller_mpc_4; 
clear controller_mpc_5; 

%% execute simulation
[T, p] = simulate_truck(T0_2,@controller_mpc_4,scen1);

%% performance verification
% diff = norm(T(:,31)-T_sp);
% disp(norm(T(:,31)-T_sp))
% disp(0.2*norm(T_sp-T0_1))
% if diff <= 0.2*norm(T_sp-T0_1)
%     disp('good performance')
% else
%     disp('bad performance')
% end