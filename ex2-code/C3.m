% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    distances = cspace;
    [~, goal_x] = min(abs(q_grid - q_goal(1)));
    [~, goal_y] = min(abs(q_grid - q_goal(2)));
    distances(goal_x, goal_y) = 2;
    list = [goal_x, goal_y];
    neighbors = [-1 0 1 1 1 0 -1 -1; 1 1 1 0 -1 -1 -1 0];

    while ~isempty(list)
        curr = list(1,:);
        curr_x = curr(1);
        curr_y = curr(2);
        dts = distances(curr_x, curr_y);
        list(1,:) = [];
        for i = 1:8
            newx = curr_x + neighbors(1, i);
            newy = curr_y + neighbors(2, i);
            if((newx > 0) && (newx < length(cspace)) && (newy > 0) && (newy < length(cspace)) && (distances(newx, newy)==0))
                distances(newx, newy) = dts + 1;
                list = [list;[newx, newy]];
            end
        end
    end
end