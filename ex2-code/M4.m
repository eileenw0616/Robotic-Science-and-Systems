% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    n = 500;
    alpha = 0.2;
    beta = 0.1;
    V = q_start;
    E = [];
    adjacency = [];
    for i = 1:n
        if rand(1) < beta
            q_target = q_goal;
        else
            q_target = M1(q_min, q_max, 1);
        end
        %q_near = nearest neighbor of q_target in V
        q_ind = knnsearch(V, q_target,"K",1);
        q_near = V(q_ind,:);
        q_new = q_near + alpha * (q_target - q_near) / norm(q_target - q_near);
        if ~check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii) && ~check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii)
            V = [V;q_new];
            E = [E;q_near, q_new];
            adjacency = [adjacency; q_ind length(V)];
        end
    end
    [neighbors, ~] = knnsearch(V, [q_start; q_goal], "K", 1);
    path_ind = shortestpath(digraph(adjacency(:,1),adjacency(:,2)), neighbors(1,1), neighbors(2,1));
    if isempty(path_ind)
        path_found = false;
    else
        path_found = true;
    end
    path = V(path_ind,:);
    path = [q_start; path];
    path = [path;q_goal];

end