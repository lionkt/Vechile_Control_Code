clc;
clear all;

%% ��������
%%%% MPC����ز���
N = 30;     %receding horizon
short_N = 10;   % short horizon
long_N = N-short_N; % long horizon
Ts = 0.01;  % MPC��������
Ts_short = 0.01;    
Ts_long = 0.2;

%%%% ������صĲ���


%%%% ��ʼ�����Ĳ���
miu = 0.85; % Ħ��ϵ��
miu_des = 0.9*miu;  % �����Ħ��������
G = 9.81;   % �������ٶ�
max_ux = 18;    % �������������ٶ�
init_ux = max_ux;    % �����longitudinal�ٶȣ�18m/s


%% Path generation
%%% ��ϸ������켣����
lambda = 25/65;         % arcռ���������ߵı���
alpha = 2*(pi/2);       % ��ֹ��ת��
d = (-0.5) - (-30.5);
z_iter = [0:0.00001:1/2];
fei_z = 2*alpha/(1+lambda).*z_iter( z_iter <= lambda/2 );   %arc�Ĳ���
fei_z = fei_z(:);
ixx = find(z_iter > lambda/2);
clothoid_part = 2*alpha/(1-lambda^2).*(-z_iter(ixx).^2 + z_iter(ixx) - lambda*lambda/4);    %��arc�Ļ��߲���
fei_z = [fei_z; clothoid_part(:)];
D_alpha_lambda = 2*trapz(z_iter, cos(fei_z));
L = d/D_alpha_lambda;       % curve���ܳ���
delta = 4*alpha/(L*L*(1-lambda*lambda));

%%% ���ݲ�������켣
total_path_length = 120;        % path���ܳ���Ϊ120m
local_curvature_max = delta*L*(1-lambda)/2;     % curvature�����ֵ
start_dist = total_path_length/2 - L/2;%curve�����λ��
end_dist = total_path_length/2 + L/2;  %curve����ֹλ��
turn_dist_1 = total_path_length/2 - L/2*lambda;
turn_dist_2 = total_path_length/2 + L/2*lambda;  %path�Ĺյ�λ��
key_pos_array = [start_dist;turn_dist_1;turn_dist_2;end_dist];
%%% �켣���ɲ���
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
    k_sn = get_curvature(now_pos,key_pos_array,delta,lambda,L);    % ��ǰ��curvature
    ax_sn_max = sqrt((miu_des*G)^2 - (ux_sn*ux_sn*k_sn)^2); % �������ṩ�������ٶ�
%     ax_sn_max = norm(ax_sn_max);
    %�Լ��ٶȽ��е���
    ux_lower = sqrt(miu_des*G/local_curvature_max);   %�ٶ��½�
    if (ux_sn > ux_lower && k_sn_pre < k_sn)    %����ʱ�ٶȴ����½�
        ax_sn = -ax_sn_max;
    end
    if (k_sn_pre > k_sn)    %����ʱ
        ax_sn = ax_sn_max;
    end
    if (ux_sn >= max_ux && ax_sn > 0)    %�ٶȴﵽ��ֵʱ
        ax_sn = 0;
    end
    next_pos = now_pos + ux_sn*Ts + ax_sn*Ts*Ts/2;  % ��һ�̵�λ��
    ux_sn_n = ux_sn + Ts*ax_sn;   % ������һ�̵��ٶ�
    now_pos = next_pos;
    ux_array = [ux_array; ux_sn_n];
    ux_sn = ux_sn_n;
    k_sn_pre = k_sn;
end
plot(ux_array);
title('speed');

%% Լ���Ĳ���





%% �Ż�����

