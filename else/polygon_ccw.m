function V2=polygon_ccw(V)
c=mean(V,1); ang=atan2(V(:,2)-c(2),V(:,1)-c(1));
[~,idx]=sort(ang); V2=V(idx,:);
end
