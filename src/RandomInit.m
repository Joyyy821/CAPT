function [X0, G] = RandomInit(N, M, R, ws)
%RANDOMINIT Randomly select initial points for robot and goals
%   INPUTS
%   N, M: number of robots and goals;
%   R: robot radius;
%   ws: workspace size;
%   OUTPUTS
%   X0: initial loactions for robots (size N x 2);
%   G: goal locations (size M x 2).

    delta = 3*R;
    if 1 % N > M
        P = random_select(M+N, ws, delta);
        % Select goal and robot from P
        ps = randperm(M+N);
        G = P(ps(1:M), :);
        X0 = P(ps(M+1:M+N), :);
    else
        X0 = random_select(N, ws, delta);
        G = random_select(M, ws, delta);
    end
end

function P = random_select(N, ws, delta)
    xlen = ws(1); ylen = ws(2);
    grid_x = floor(xlen/delta); 
    grid_y = floor(ylen/delta);
    margin_x = (xlen - grid_x * delta) / 2;
    margin_y = (ylen - grid_y * delta) / 2;
    % Random select N grids
    N_grid = grid_x * grid_y;
    ps = randperm(N_grid, N);
    P = zeros(N, 2);
    for i=1:N
        p_i = ps(i);
        p_iy = ceil(p_i/grid_x);
        p_ix = p_i - (p_iy-1)*grid_x;
        P(i, 1) = (p_ix - 0.5) * delta + margin_x;
        P(i, 2) = (p_iy - 0.5) * delta + margin_y;
    end
end
