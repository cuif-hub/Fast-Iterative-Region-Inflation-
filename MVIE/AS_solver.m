function x_opt = AS_solver(x_pre,cK,A,c,d,tao)
%AS 求解大量约束的SOCP问题：min cK'x s.t.(ci'x+di,xi'Ai)∈Kni, 1≤i≤mbar
%                             x_pre是上一次迭代得到的最优解

Hfi = 0;
for i = 1:length(A)
    fi = (c{i}'*x_pre+d{i})^2 - x_pre'*(A{i}*A{i}')*x_pre;
    del_fi = 2*((c{i}'*x_pre+d{i})*c{i} - A{i}*A{i}'*x_pre);
    del2_fi = 2 * (c{i}*c{i}' - A{i}*A{i}');
    Hfi = Hfi + del_fi*del_fi'/(fi^2) - del2_fi/fi;
end

x_opt = x_pre - tao * (Hfi\cK)/sqrt(cK'*(Hfi\cK));

end

