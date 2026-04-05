function [Aout, bout] = RsI(E, Q, obstacles, n)
% Restrictive Inflation: compute halfspaces for obstacles

Aout = []; bout = [];
L_E = E.L; b_E = E.b;
invMap = @(X) (L_E \ (X' - b_E))'; %标准化函数

% Step 1: 在标准化空间中构建半空间
N = numel(obstacles);
a_list = zeros(n,N);
rhs_list = zeros(N,1);
Obar_list = cell(N,1);

Qbar = invMap(Q);
for i=1:N
    Obar = invMap(obstacles{i});
    Obar_list{i} = Obar;
    
    % Solve SDMN: min ||y||^2 s.t. v^T y <= 1, u^T y >= 1
    E_mat = [Qbar; -Obar];
    f_vec = [ones(size(Qbar,1),1); -ones(size(Obar,1),1)];
    b = SDMN_solve(E_mat, f_vec, n);
    
    denom = b'*b;
    if denom <= 0, continue; end
    
    ai = b/denom;         % in normalized space
    rhs = 1/denom;         % halfspace: a_i^T x <= rhs
    a_list(:,i) = ai;
    rhs_list(i) = rhs;
end

% step2 贪心选取候选半空间 (Algorithm 1: 13–17)
I = 1:N;
Pbar.A = []; Pbar.b = [];
while ~isempty(I)
    [~, minIdx] = min(rhs_list(I));
    j = I(minIdx);
    Pbar.A = [Pbar.A; a_list(:,j)']; Pbar.b = [Pbar.b; rhs_list(j)];
    I(minIdx) = [];

    %更新I，移除那些已经在该半空间外的障碍物及对应的半空间
    new_I = [];
    for k = 1:length(I)
        idx = I(k);
        Obar = Obar_list{idx};
        if ~all(Obar * a_list(:,j) >= rhs_list(j))
            new_I(end+1) = idx;
        end
    end
    I = new_I;
end

% Step 3: 映射回原始空间
Aout = []; bout = [];
for i=1:size(Pbar.A,1)
    aiT = Pbar.A(i,:); rhs = Pbar.b(i);
    % 原空间半空间: (ai^T * L^-1)(x - bE) <= rhs
    Arow = (L_E' \ aiT')';
    bval = rhs + Arow * b_E;
    norm_Arow = norm(Arow);
    Aout = [Aout; Arow/norm_Arow]; %将公式标准化，公式更直观
    bout = [bout; bval/norm_Arow];
end


end
