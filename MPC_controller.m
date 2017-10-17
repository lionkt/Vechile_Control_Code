clc;
clear all;

%% MPC的相关参数
N = 30;     %receding horizon
short_N = 10;   % short horizon
long_N = N-short_N; % long horizon
Ts = 0.01;  % MPC控制周期
Ts_short = 0.01;    
Ts_long = 0.2;



%% Path generation
start_dist = 30;end_dist = 90;  %path 的起止位置
turn_dist_1 = 50;turn_dist_2 = 70;  %path的拐点位置
key_pos_array = [start_dist;turn_dist_1;turn_dist_2;end_dist];
local_curvature_max = 0.07;
local_curve_length = 70;
delta = local_curvature_max/(turn_dist-start_dist);   
lambda = 1-2*local_curvature_max/(delta*local_curve_length);





%% 约束的参数





%% 优化计算

