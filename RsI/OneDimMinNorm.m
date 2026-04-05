function y = OneDimMinNorm(E_mat,f_vec)
ub_idx = find(E_mat>0);
lb_idx = find(E_mat<0);

if ~isempty(ub_idx) && ~isempty(lb_idx)
    min_ub = min(f_vec(ub_idx)./E_mat(ub_idx));
    max_lb = max(f_vec(lb_idx)./E_mat(lb_idx));
    if min_ub < max_lb
        y = [];
    elseif max_lb * min_ub <= 0
        y = 0;
    elseif max_lb > 0
        y = max_lb;
    else
        y = min_ub;
    end
elseif isempty(ub_idx) && ~isempty(lb_idx)
    max_lb = max(f_vec(lb_idx)./E_mat(lb_idx));
    if max_lb <= 0
        y = 0;
    else
        y = max_lb;
    end
elseif ~isempty(ub_idx) && isempty(lb_idx)
    min_ub = min(f_vec(ub_idx)./E_mat(ub_idx));
    if min_ub >= 0
        y = 0;
    else
        y = min_ub;
    end
else %isempty(ub_idx) && isempty(lb_idx) 即无约束
   y = 0; 
end

end

