function [V,isClosed]=intersectHalfspaces2D(Ai,bi)
N=size(Ai,1); V=[];
for i=1:N-1
    for j=i+1:N
        A2=[Ai(i,:);Ai(j,:)]; b2=[bi(i);bi(j)];
        if rank(A2)<2, continue; end
        p=A2\b2;
        if all(Ai*p<=bi+1e-9), V=[V;p']; end
    end
end
if isempty(V), isClosed=false; return; end
V=unique(round(V*1e12)/1e12,'rows');
if size(V,1)<3, isClosed=false; return; end
V=polygon_ccw(V); isClosed=true;
end
