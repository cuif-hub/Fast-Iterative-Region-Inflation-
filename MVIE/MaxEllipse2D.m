function E = MaxEllipse2D(P)
A = P.A; b = P.b; [m,n] = size(A); assert(n==2);
bestE = []; bestArea = -Inf;

for N=3:5
    if nchoosek(m,N) > 1000, continue; end
    C = nchoosek(1:m,N);
    for r=1:size(C,1)
        idx = C(r,:); Ai=A(idx,:); bi=b(idx);
        [V,isClosed] = intersectHalfspaces2D(Ai,bi);
        if ~isClosed, continue; end
        try, Ebar = MENN_Ngon(Ai,bi); catch, continue; end
        area = pi*prod(diag(Ebar.D));
        if area>bestArea, bestArea=area; bestE=Ebar; end
    end
end

if isempty(bestE)
    [c,r] = chebyshev_center(A,b);
    E.A=eye(2); E.D=r*eye(2); E.b=c;
else
    E=bestE;
end
end
