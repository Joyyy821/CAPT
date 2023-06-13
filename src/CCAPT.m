function [gamma_x, gamma_y, G] = CCAPT(X0, G, vmax, dt)
%CCAPT Summary of this function goes here

[N, ~] = size(X0);
[M, ~] = size(G);
%% (1) Find optimal assignment by solving linear assignment problem
% Create the D matrix (cost)
D = zeros(N, M);
for i=1:N
    for j=1:M
        D(i, j) = norm(X0(i, :) - G(j, :))^2;
    end
end

% Solve the linear assignment problem
unassign_cost = 1e3;
while true
    [pairs, uR, uG] = matchpairs(D, unassign_cost);
    if isempty(uR) || isempty(uG)
        break
    end
end
PHI = zeros(N, M);
for i=1:min(M, N)
    PHI(pairs(i,1), pairs(i,2)) = 1;
end

%% (2) Calculate the finishing time and coefficients in beta(t)
tf = 0;
for i=1:min(M, N)
    ti = norm(X0(pairs(i,1), :) - G(pairs(i,2), :))/vmax;
    tf = max(ti, tf);
end
t0 = 0;
alpha_0 = -t0/(tf-t0); alpha_1 = 1/(tf-t0);

%% (3) Calculate the optimal trajectory
T = [t0:dt:tf]';
beta_t = alpha_0 + alpha_1*T;
gamma_x = (1-beta_t) * X0(:, 1)' + ...
    beta_t * (PHI*G(:, 1) + (eye(N) - PHI*PHI') * X0(:, 1))';
gamma_y = (1-beta_t) * X0(:, 2)' + ...
    beta_t * (PHI*G(:, 2) + (eye(N) - PHI*PHI') * X0(:, 2))';

% Update G (to the same index order as X0)
G = PHI * G;
if N >= M
    for i=1:N
        gi = G(i, :);
        if ~any(gi)
            G(i, :) = X0(i, :);
        end
    end
end
end
