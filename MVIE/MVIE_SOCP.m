function [Enew,tnew,snew] = MVIE_SOCP(P,E,t_pre,s_pre,n,tao)

if n == 2
    nbar = n*(n+3)/2 + 1; %决策变量:Lε的下三角元素，共n(n+1)/2;
                      %        bε的所有元素，共n个;
                      %        t，共1个;
    m = size(P.A,1); %(35c)对应的约束个数
    mbar = m + 1; %SOCP约束:(34b)，共1个;
                      %     (35c)，共m个.

    %(35a)中的cK
    cK = zeros(nbar,1); cK(end) = -1;

    % 约束函数转换
    A = cell(1,mbar); c = cell(1,mbar); d = cell(1,mbar);

    %(34c)->(35b)
    for i = 1:m
        PA = P.A; Pb = P.b;
        tempA = zeros(nbar,n); tempA(1:2,1)=PA(i,:)'; tempA(3,2)=PA(i,2);
        A{i} = tempA;
        c{i} = [zeros(1,n*(n+1)/2),-PA(i,:),0]';
        d{i} = Pb(i);
    end
    
    %(34b)->(35b)
    tempA = zeros(nbar,2); tempA(end,1)=2; tempA(1,2)=1; tempA(3,2)=-1;
    A{m+1} = tempA;
    tempc = zeros(nbar,1); tempc(1)=1; tempc(3)=1;
    c{m+1} = tempc;
    d{m+1} = 0;

    L_pre = E.L;
    x_pre = [L_pre(tril(true(size(L_pre))));E.b;t_pre];

    x_opt = AS_solver(x_pre,cK,A,c,d,tao);

    Lnew = [x_opt(1:2), [0; x_opt(3)]];
    bnew = x_opt(n*(n+1)/2+1:end-1);
    Enew.L = Lnew; Enew.b = bnew;
    tnew = x_opt(end);
    snew = s_pre;
else %n=3
    % 需要修改，先别用
    nbar = n*(n+5)/2; %决策变量:Lε的下三角元素，共n(n+1)/2;
                  %        bε的所有元素，共n个;
                  %        t，共1个;
                  %        辅助变量s(1)~s(n-1)，共n-1个.
    m = size(P.A,1); %(35c)对应的约束个数
    mbar = m + n; %SOCP约束:(34b)，共n个;
              %         (35c)，共m个.

    %(35a)中的cK
    cK = zeros(nbar,1); cK(end-n+1) = -1;

    % 约束函数转换
    A = cell(1,mbar); c = cell(1,mbar); d = cell(1,mbar);

    %(34c)->(35b)
    for i = 1:m
        PA = P.A; Pb = P.b;
        tempA = zeros(nbar,n); tempA(1:3,1)=PA(i,:)'; tempA(4:5,2)=PA(i,2:3)'; tempA(6,2)=PA(i,3);
        A{i} = tempA;
        c{i} = [zeros(1,n*(n+1)/2),-PA(i,:),zeros(1,n)]';
        d{i} = Pb(i);
    end
    
    %(34b)->(35b) 这段需要修改，用的时候注意
    tempA = zeros(nbar,2); tempA(end-1,1)=sqrt(2); tempA(1,2)=1; tempA(3,2)=-0.5;
    A{m+1} = tempA;
    tempc = zeros(nbar,1); tempc(1)=1; tempc(2)=0.5;
    c{m+1} = tempc;
    d{m+1} = 0;

    tempA = zeros(nbar,2); tempA(end,1)=sqrt(2); tempA(end-1,2)=1; tempA(6,2)=-0.5;
    A{m+2} = tempA;
    tempc = zeros(nbar,1); tempc(end-1)=1; tempc(6)=0.5;
    c{m+2} = tempc;
    d{m+2} = 0;

    tempA = zeros(nbar,1); tempA(end-2)=1;
    A{m+3} = tempA;
    tempc = zeros(nbar,1); tempc(end)=1;
    c{m+3} = tempc;
    d{m+3} = 0;

    L_pre = E.L;
    x_pre = [L_pre(tril(true(size(L_pre))));E.b;t_pre;s_pre];

    x_opt = AS_solver(x_pre,cK,A,c,d,tao);

    Lnew = x_opt(1:end-2*n); bnew = x_opt(end-2*n+1:end-n);
    Enew.L = Lnew; Enew.b = bnew;
    tnew = x_opt(end-2);
    snew = x_opt(end-1:end);
end

end
