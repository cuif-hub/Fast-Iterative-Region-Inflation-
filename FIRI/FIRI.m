function [P,E] = FIRI(seed_Q, obstacles, E0, opts)
% Fast Iterative Region Inflation (FIRI)
% seed_Q: s x n vertices of seed polytope Q
% obstacles: cell array, each obstacle is r x n vertices
% E0: initial ellipsoid struct {A, D, b}
% opts: struct('maxIter',50,'tol',1e-6,'nDim',2,'verbose',1)
% returns P: polytope in H-rep {A,b}

if nargin<4, opts = struct(); end
if ~isfield(opts,'maxIter'), opts.maxIter = 50; end
if ~isfield(opts,'tol'), opts.tol = 1e-6; end
if ~isfield(opts,'nDim'), opts.nDim = size(seed_Q,2); end

n = opts.nDim;
E = E0; % ellipsoid
P.A = []; P.b = [];
prev_vol = prod(diag(E.L));
t = sqrt(prev_vol)/2;
s = zeros(n-1,1); %决策变量中的辅助变量
tao = 0.5;

for k = 1:opts.maxIter    
    [Ahalf, bh] = RsI(E, seed_Q, obstacles, n);
    P.A = Ahalf; P.b = bh;
    
    [Enew,tnew,snew] = MVIE_SOCP(P,E,t,s,n,tao);
    
    vol = prod(diag(Enew.L));
    
    if abs(vol - prev_vol) < opts.tol * prev_vol
        fprintf('Iter = %d , Converged.', k);
        break;
    end
    prev_vol = vol;
    E = Enew;
    t = tnew;
    s = snew;
end
fprintf('Iter = %d , non-Converged.', k);

end

