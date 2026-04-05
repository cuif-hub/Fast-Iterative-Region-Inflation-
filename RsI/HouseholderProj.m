function [M,v,Ered,fred] = HouseholderProj(E_mat,f_vec,I,idx)
[~,n] = size(E_mat);
e = E_mat(idx,:); fi = f_vec(idx);

% Householder变换
denom = e*e';
if abs(denom) < 1e-15
    % 处理退化情况
    M = eye(n-1);
    v = zeros(n,1);
    Ered = [];
    fred = [];
    return;
end

v = (fi/denom) * e';
[~,j] = max(abs(v)); 
ej = zeros(n,1); ej(j) = 1;
u = v + sign(v(j)) * norm(v) * ej;
H = eye(n) - 2/(u'*u) * (u*u');
M = H'; M(:,j) = [];

% 降维子问题的约束
if isempty(I)
    Ered = [];
    fred = [];
else
    Ered = E_mat(I,:) * M;
    fred = f_vec(I) - E_mat(I,:) * v;
end

end

