% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    adjacency = zeros(num_samples);
    samples = zeros(num_samples, 4);
    for i = 1:num_samples
        sample = q_min + ((q_max - q_min)) .* rand(1, 4);
        while check_collision(robot, sample, link_radius, sphere_centers, sphere_radii)
            sample = q_min + ((q_max - q_min)) .* rand(1, 4);
        end
        samples(i,:) = sample;
    end
    [index, dts] = knnsearch(samples, samples, 'K', num_neighbors);
    %https://www.mathworks.com/help/stats/knnsearch.html
    for j = 1:num_samples
        for k = 1:num_neighbors
            l = index(j, k);
            if ~check_edge(robot, samples(l,:), samples(j,:), link_radius, sphere_centers, sphere_radii)
                adjacency(l, j) = dts(j, k);
                adjacency(j, l) = dts(j, k);
            end
        end
    end
end