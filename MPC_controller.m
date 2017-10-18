clc;
clear all;

%% 参数设置
%%%% MPC的相关参数
N = 30;     %receding horizon
short_N = 10;   % short horizon
long_N = N-short_N; % long horizon
Ts = 0.01;  % MPC控制周期
Ts_short = 0.01;    
Ts_long = 0.2;

%%%% 车辆相关的参数


%%%% 初始条件的参数
miu = 0.85; % 摩擦系数
miu_des = 0.9*miu;  % 理想的摩擦利用率
G = 9.81;   % 重力加速度
max_ux = 18;    % 车辆纵向的最大速度
init_ux = max_ux;    % 进入的longitudinal速度，18m/s


%% Path generation
%%% 精细化计算轨迹参数
lambda = 25/65;         % arc占据整个弧线的比例
alpha = 2*(pi/2);       % 起止点转角
d = (-0.5) - (-30.5);
z_iter = [0:0.00001:1/2];
fei_z = 2*alpha/(1+lambda).*z_iter( z_iter <= lambda/2 );   %arc的部分
fei_z = fei_z(:);
ixx = find(z_iter > lambda/2);
clothoid_part = 2*alpha/(1-lambda^2).*(-z_iter(ixx).^2 + z_iter(ixx) - lambda*lambda/4);    %非arc的弧线部分
fei_z = [fei_z; clothoid_part(:)];
D_alpha_lambda = 2*trapz(z_iter, cos(fei_z));
L = d/D_alpha_lambda;       % curve的总长度
delta = 4*alpha/(L*L*(1-lambda*lambda));

%%% 根据参数计算轨迹
total_path_length = 120;        % path的总长度为120m
local_curvature_max = delta*L*(1-lambda)/2;     % curvature的最大值
start_dist = total_path_length/2 - L/2;%curve的起点位置
end_dist = total_path_length/2 + L/2;  %curve的终止位置
turn_dist_1 = total_path_length/2 - L/2*lambda;
turn_dist_2 = total_path_length/2 + L/2*lambda;  %path的拐点位置
key_pos_array = [start_dist;turn_dist_1;turn_dist_2;end_dist];
%%% 轨迹生成测试
ux_array = init_ux;
ux_sn = init_ux;
k_sn_pre = 0;
now_pos = 0;
ax_sn = 0;
pos_array = [0:0.001:120];
curvature_arr = zeros(length(pos_array),1);
figure;
for i=1:length(pos_array)
    k_sn = get_curvature(pos_array(i),key_pos_array,delta,lambda,L);
    curvature_arr(i) = k_sn;
end
plot(pos_array,curvature_arr);
title('curvature');

figure;
while(now_pos < total_path_length)
    k_sn = get_curvature(now_pos,key_pos_array,delta,lambda,L);    % 当前的curvature
    ax_sn_max = sqrt((miu_des*G)^2 - (ux_sn*ux_sn*k_sn)^2); % 环境能提供的最大加速度
%     ax_sn_max = norm(ax_sn_max);
    %对加速度进行调整
    ux_lower = sqrt(miu_des*G/local_curvature_max);   %速度下界
    if (ux_sn > ux_lower && k_sn_pre < k_sn)    %入弯时速度大于下界
        ax_sn = -ax_sn_max;
    end
    if (k_sn_pre > k_sn)    %出弯时
        ax_sn = ax_sn_max;
    end
    if (ux_sn >= max_ux && ax_sn > 0)    %速度达到最值时
        ax_sn = 0;
    end
    next_pos = now_pos + ux_sn*Ts + ax_sn*Ts*Ts/2;  % 下一刻的位置
    ux_sn_n = ux_sn + Ts*ax_sn;   % 计算下一刻的速度
    now_pos = next_pos;
    ux_array = [ux_array; ux_sn_n];
    ux_sn = ux_sn_n;
    k_sn_pre = k_sn;
end
plot(ux_array);
title('speed');

%% 约束的参数





%% 优化计算

