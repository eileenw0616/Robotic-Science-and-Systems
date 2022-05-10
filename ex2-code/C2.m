% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
    [~, n] = size(q_grid);
    cspace = zeros(n);
    for i = 1:n
        for j = 1:n
            [poly1, poly2, ~, ~] = q2poly(robot, [q_grid(i);q_grid(j)]);
            for k = 1:5
                obs1 = intersect(poly1, obstacles(k));
                obs2 = intersect(poly2, obstacles(k));
                if obs1.NumRegions > 0 || obs2.NumRegions > 0
                    cspace(i, j) = 1;
                end
            end
        end
    end


end