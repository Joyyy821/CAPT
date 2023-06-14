function [X0, G] = RandomInit(N, M, R, ws)
%RANDOMINIT Randomly select initial points for robot and goals
%   Author: Yang Jiao (UCSD)
%
%   INPUTS
%   N, M: number of robots and goals;
%   R: robot radius;
%   ws: workspace size;
%   OUTPUTS
%   X0: initial loactions for robots (size N x 2);
%   G: goal locations (size M x 2).

    delta = 3*R;

    P = random_select(M+N, ws, delta);
    % Select goal and robot from P
    ps = randperm(M+N);
    G = P(ps(1:M), :);
    X0 = P(ps(M+1:M+N), :);
end

function P = random_select(N, ws, delta)
%RANDOM_SELECT Randomly select N (grid) points in a given workspace with
%certain clearance distance (grid length) between sampled points
%   INPUTS
%   N: number of items to sample;
%   ws: workspace size;
%   delta: clearance distance;
%   OUTPUT
%   P: sample coordinates (size N x 2).

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
