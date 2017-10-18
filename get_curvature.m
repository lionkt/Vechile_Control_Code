function [ now_curvature ] = get_curvature( now_pos, key_pos_array, delta, lambda, curve_length )
% ����path�����ʣ���������������һ��λ�õ�
%%% ����Collision Avoidance and Stabilization for Autonomous Vehicles in
%%% Emergency Scenarios���õĲ���������ķ�������Collision Avoidance and 
%%% Stabilization for Autonomous Vehicles in Emergency Scenarios
% ���������
% key_pos_array = [start_dist;turn_dist_1;turn_dist_2;end_dist]

if now_pos < key_pos_array(1)
    now_curvature = 0;
elseif now_pos < key_pos_array(2)
    now_curvature = delta*(now_pos - key_pos_array(1)); % ����Բ��ǰ
elseif now_pos < key_pos_array(3)
    now_curvature = delta*curve_length*(1-lambda)/2;    % Բ����
elseif now_pos < key_pos_array(4)
    now_curvature = delta*(curve_length-(now_pos- key_pos_array(1)));   % ��Բ��
else
    now_curvature = 0;
end
    
end

