% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    n = length(path);
    smoothed_path = zeros(n,4);
    smoothed_path(1,:) = path(1,:);
    q_goal = path(end,:);
    curr = 1;
    while curr < n
        for i = n:-1:curr
            if ~check_edge(robot, path(curr,:), path(i,:), link_radius, sphere_centers, sphere_radii)
                smoothed_path = [smoothed_path; path(i,:)];
                curr = i;
                break;
            end
        end
    end
end