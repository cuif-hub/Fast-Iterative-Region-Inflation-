function y = SDMN_solve(E_mat, f_vec, n)
% Solve min ||y||^2 s.t. E_mat*y <= f_vec using Seidel-like recursion
[d,~] = size(E_mat);
% 无约束时
if d == 0, y = zeros(n,1); return; end

% n=1时
if n == 1, y = OneDimMinNorm(E_mat,f_vec); return; end

y = zeros(n,1);
% 随机排列`约束顺序
constraint_order = randperm(d);
I = []; %已处理过的约束集合
for i=1:d
    idx = constraint_order(i);
    e = E_mat(idx,:); fi = f_vec(idx);
    % y属于h
    if e*y <= fi + 1e-12, I = [I, idx]; continue; end
    % y违反h
    % Householder变换
    [M,v,Ered,fred] = HouseholderProj(E_mat,f_vec,I,idx);

    % solve reduced problem
    yprime = SDMN_solve(Ered, fred, n-1);
    y = M*yprime + v;
    I = [I, idx]; %将已处理的约束加入到约束集合中
end

end
