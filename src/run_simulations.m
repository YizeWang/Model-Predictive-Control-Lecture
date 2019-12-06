% Author: Yize Wang
% Email: yizwang@student.ethz.ch

%% init
clear all
close all
clc
addpath(genpath(cd));
load('system/parameters_scenarios.mat');
param = compute_controller_base_parameters;

T_sp = param.T_sp;
T0_1 = T_sp + [3;1;0];
T0_2 = T_sp + [-1;-0.1;-4.5];

%% clear persistent variables
clear controller_lqr; 
clear controller_mpc_1; 
clear controller_mpc_2; 
clear controller_mpc_3; 
clear controller_mpc_4; 
clear controller_mpc_5; 

%% execute simulation
[T, p] = simulate_truck(T0_2,@controller_mpc_4,scen1);