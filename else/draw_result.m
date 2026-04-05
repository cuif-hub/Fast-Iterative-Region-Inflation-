function draw_result(P,E0,E,seed,env_size,obstacles)
% Visualization
figure('Position',[50 50 800 750]); hold on; axis equal; grid on;

% Draw obstacles
if ~isempty(obstacles)
    for i=1:numel(obstacles)-4
        V=obstacles{i}; fill(V(:,1),V(:,2),[1,0.6,0.6],'FaceAlpha',0.5,'EdgeColor','r');
        % 计算多边形的中心点
        center = mean(V, 1);
        % 在多边形中心添加数字标识
        text(center(1), center(2), num2str(i), ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'middle', ...
             'FontSize', 10, ...
             'FontWeight', 'bold', ...
             'Color', 'black');
    
    end
end

% Draw initial ellipsoid
if ~isempty(E0)
    ellipse_initial = draw_ellipse(E0.L, E0.b, 'r');
end

% Draw inflated polytope
if ~isempty(P)
    [V,isClosed]=intersectHalfspaces2D(P.A,P.b);
    if isClosed
        inflation_polygon_fill = fill(V(:,1),V(:,2),[0.6,0.8,1],'FaceAlpha',0.4,'EdgeColor','b','LineWidth',2);
    end
end

% Draw final ellipsoid
if ~isempty(E)
    ellipse_final = draw_ellipse(E.L, E.b, 'g');
end

% Draw seed point
if ~isempty(seed)
    seed_plot = plot(seed(1),seed(2),'ko','MarkerFaceColor','k','MarkerSize',4);
end

% legend([inflation_polygon_fill,seed_plot,ellipse_initial,ellipse_final],{'Inflated Polytope','Seed point','ε_0','ε_final'});

% 设置坐标轴范围和刻度
xlim([0, env_size(1)]);
ylim([0, env_size(2)]);
set(gca, 'XTick', 0:50:env_size(1), 'YTick', 0:50:env_size(2));


end

