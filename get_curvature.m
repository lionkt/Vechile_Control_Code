function [ now_curvature ] = get_curvature( now_pos, key_pos_array, delta, lambda, curve_length )
% 计算path的曲率：输入输出都是针对一个位置点
%%% 根据Collision Avoidance and Stabilization for Autonomous Vehicles in
%%% Emergency Scenarios设置的参数，具体的方程来自Collision Avoidance and 
%%% Stabilization for Autonomous Vehicles in Emergency Scenarios
% 输入参数：
% key_pos_array = [start_dist;turn_dist_1;turn_dist_2;end_dist]

if now_pos < key_pos_array(1)
    now_curvature = 0;
elseif now_pos < key_pos_array(2)
    now_curvature = delta*(now_pos - key_pos_array(1)); % 进入圆弧前
elseif now_pos < key_pos_array(3)
    now_curvature = delta*curve_length*(1-lambda)/2;    % 圆弧段
elseif now_pos < key_pos_array(4)
    now_curvature = delta*(curve_length-(now_pos- key_pos_array(1)));   % 出圆弧
else
    now_curvature = 0;
end
    
end

