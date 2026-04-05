clear; clc;

seed_Q = [30,30];
% Initial ellipsoid (tiny ball around the seed)
E0.L = 0.05*eye(2);
E0.b = seed_Q';   % center at the seed


% 生成障碍物和边界障碍物
env_size = [0,50,0,50];
Nobstacles = 20;
obstacles = getObstacles(env_size,Nobstacles);
env_size = env_size([2,4]);
obstacles{end+1} = [-1 -1;env_size(1)+1 -1;env_size(1)+1 0;-1 0];
obstacles{end+1} = [env_size(1) -1;env_size(1)+1 -1;env_size(1)+1 env_size(2)+1;env_size(1) env_size(2)+1];
obstacles{end+1} = [env_size(1)+1 env_size(2);env_size(1)+1 env_size(2)+1;-1 env_size(2)+1;-1 env_size(2)];
obstacles{end+1} = [-1 -1;0 -1;0 env_size(2)+1;-1 env_size(2)+1]; 

% Run FIRI
% 如果算法报错了，可能是种子在障碍物内
opts=struct('maxIter',30,'tol',1e-6,'nDim',2,'verbose',1);

[P,E] = FIRI(seed_Q, obstacles, E0, opts);

draw_result(P,E0,E,seed_Q,env_size,obstacles);