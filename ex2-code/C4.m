% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    [~, goal_x] = min(abs(q_grid - q_start(1)));
    [~, goal_y] = min(abs(q_grid - q_start(2)));
    path = [goal_x, goal_y];
    neighbors = [-1 0 1 1 1 0 -1 -1; 1 1 1 0 -1 -1 -1 0];
    while distances(path(end, 1), path(end, 2)) > 2
        curr_x = path(end, 1);
        curr_y = path(end, 2);
        curr_dts = distances(curr_x, curr_y);
        for i = 1:8
            newx = curr_x + neighbors(1, i);
            newy = curr_y + neighbors(2, i);
            dts = distances(newx, newy);
            if dts > 1 && dts < curr_dts
                curr_x = newx;
                curr_y = newy;
                curr_dts = dts;
            end
        end
        path(end+1, :) = [curr_x, curr_y];
    end
end