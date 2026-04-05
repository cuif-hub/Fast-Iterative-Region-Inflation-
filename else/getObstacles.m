function Obstacles = getObstacles(bound,Nobstacles)
%[bound,Obstacles] = getObstacles(Nobstacles)
% 根据输入获得环境边界和障碍物V-resp(随机生成)
% INPUTS:
%      xmax = scalar = 环境最大x轴长度
%      ymax = scalar = 环境最大y轴长度
%      Nobstacles = scalar = 环境中障碍物的个数
% OUTPUTS:
%      bound = [1,4] = [xmin xmax ymin ymax]
%      Obstacles = cell = 障碍物V-resp(一列为一顶点)

radius_range = [2,4];

rng(1); % 固定随机数种子以确保每次运行生成相同障碍物
Obstacles = cell(1, Nobstacles);
obstacle_centers = [];

max_vertices = 6; % 每个障碍物最多顶点数
min_vertices = 3; % 每个障碍物最少顶点数
min_distance = 8; % 障碍物之间的最小距离，避免重叠

% 边界安全距离，确保障碍物不会超出边界
boundary_margin = 2;

for i = 1:Nobstacles
    is_valid = false;  % 判断障碍物是否有效（无重叠）
    attempts = 0;
    max_attempts = Nobstacles * 50; % 防止无限循环
    
    % 确保出发区和目标区不与障碍物重叠
    while ~is_valid && attempts < max_attempts
        attempts = attempts + 1;
        
        % 随机生成障碍物中心位置，考虑边界安全距离
        center_x = randi([bound(1) + boundary_margin, bound(2) - boundary_margin]);
        center_y = randi([bound(3) + boundary_margin, bound(4) - boundary_margin]);

        % 判断是否与已有障碍物重叠
        is_valid = true;
        for j = 1:size(obstacle_centers, 1)
            dist = sqrt((center_x - obstacle_centers(j, 1))^2 + (center_y - obstacle_centers(j, 2))^2);
            if dist < min_distance
                is_valid = false;
                break;
            end
        end
        
        if ~is_valid
            continue;
        end
        
        % 随机生成障碍物顶点数
        num_vertices = randi([min_vertices, max_vertices]);

        % 随机生成障碍物的顶点，确保在边界内
        theta = linspace(0, 2*pi, num_vertices + 1);
        % 计算最大允许半径，确保顶点不超出边界
        max_possible_radius_x = min(center_x - bound(1), bound(2) - center_x);
        max_possible_radius_y = min(center_y - bound(3), bound(4) - center_y);
        max_possible_radius = min(max_possible_radius_x, max_possible_radius_y) - 1; % 减1作为安全余量
        
        % 限制半径范围在3到最大允许半径之间
        radius_range_min = radius_range(1);
        radius_range_max = min(5, radius_range(2));
        
        if radius_range_max < radius_range_min
            is_valid = false;
            continue;
        end
        
        radius = randi([radius_range_min, radius_range_max], 1, num_vertices);
        x = radius .* cos(theta(1:end-1)) + center_x; % X坐标
        y = radius .* sin(theta(1:end-1)) + center_y; % Y坐标
        
        % 确保所有顶点都在边界内
        if any(x < bound(1)) || any(x > bound(2)) || any(y < bound(3)) || any(y > bound(4))
            is_valid = false;
            continue;
        end
        
        % 计算凸包，防止顶点乱七八糟
        K = convhull(x, y); % 求凸包的索引
        Obstacles{i} = [x(K); y(K)]'; % 存储凸包的坐标
        Obstacles{i}(end,:) = [];

        % 更新障碍物中心位置
        obstacle_centers = [obstacle_centers; center_x, center_y]; 
    end

end

