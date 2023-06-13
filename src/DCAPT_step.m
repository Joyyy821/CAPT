function [X_next,F_next, G_R, U, is_stop] = DCAPT_step(X, F, h, vmax, dt, U, G_prev)
%DCAPT_step Summary of this function goes here
%   Detailed explanation goes here
    G_R = getProximity(X, h);
    if nargin == 5
        X_next = X; F_next = F; U = G_R; is_stop = false;
        return
    end
    if nargin ~= 7
        error("Wrong number of input argument. Expect 7 but got "...
            +string(nargin));
    end
    [N, ~] = size(X);
    PHI = eye(N);
    switch_pairs = [];
    for i=1:N
        Ci = G_R(i, :);
        Ci_prev = G_prev(i, :);
        for j=1:N
            if Ci(j) && ~Ci_prev(j)
                U(i, j) = 1;
            end
        end
        U(i, :) = U(i, :) & Ci;
%         Ui = U(i, :);
        
        xi = X(i, :); fi = F(i, :);
%         sp = [0, 0];
        for j=1:N
            if U(i, j)
                xj = X(j, :); fj = F(j, :);
                uij = xj - xi; wij = fj - fi;
                if uij * wij' < 0
                    % switch goal signal
%                     sp = [i, j];
                    switch_pairs = [switch_pairs; i, j];
%                     k = logical(PHI(i, :));
%                     PHI(i, j) = 1; PHI(i, k) = 0;
%                     l = logical(PHI(j, :));
%                     PHI(j, i) = 1; PHI(j, l) = 0;
                    U(i, :) = Ci;
%                     Ui = Ci;
                end
                U(i, j) = 0;
%                 end
            end
        end
%         if any(sp)
%             switch_pairs = [switch_pairs; sp];
%         end
    end
    % Perform goal switching
    if ~isempty(switch_pairs)
        disp(switch_pairs)
%     [K, ~] = size(switch_pairs);
        ls = [];
        for k=1:N
    %         l = logical(PHI(switch_pairs(k,1), :));
    %         PHI(switch_pairs(k,1), switch_pairs(k,2)) = 1;
    %         PHI(switch_pairs(k,1), l) = 0;
            if ~isempty(find(ls==k, 1))
                continue;
            end
            idx = find(switch_pairs(:,1)==k, 1);
            if ~isempty(idx)
                l = switch_pairs(idx, 2);
%                 if ~isempty(find(ls==l, 1))
%                     continue;
%                 end
                ls = [ls, l];
                m = logical(PHI(k, :));
                n = logical(PHI(l, :));
%                 p = logical(PHI(n, :));
                PHI(k, n) = 1; PHI(k, m) = 0;
                PHI(l, k) = 1; PHI(l, n) = 0;
            end
        end
    end
    
    F_next = PHI * F;
    X_next = zeros(N, 2);
    is_stop = zeros(N);
    % Get trajectory
    for i=1:N
        xi = X(i, :);
        fi = F_next(i, :);
        if norm(fi - xi)
            d = vmax*dt/norm(fi - xi);
            d = min(d, 1);  % The robot can't go further than the goal
            if d == 1
                is_stop(i) = 1;
            end
            X_next(i, :) = (1 - d) * xi + d * fi;
        else
            X_next(i, :) = xi;
            is_stop(i) = 1;
        end
    end
    is_stop = all(is_stop);
end

function G_R = getProximity(X, h)
% TODO comments
    [N, ~] = size(X);
    G_R = zeros(N);
    for i=1:N
        for j=1:N
            xi = X(i, :); xj = X(j, :);
            if norm(xi-xj) <= h && i~=j
                % i and j are neighbors
                G_R(i, j) = 1; G_R(j, i) = 1;
            end
        end
    end
end