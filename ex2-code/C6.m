% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)
	num_collisions = 0;
	n = length(q_path);
	for i = n - 1
		[poly1, poly2, ~, ~] = q2poly(robot, q_path(i, :));
		[poly3, poly4, ~, ~] = q2poly(robot, q_path(i+1, :));
		link1 = union(poly1, poly2);
		link2 = union(poly3, poly4);
		link1 = convhull(link1);
		link2 = convhull(link2);
        for obs = obstacles
            cls1 = overlaps(obs, link1);
            cls2 = overlaps(obs, link2);
            if cls1 || cls2
                C1(robot, q_path(i,:));
                C1(robot, q_path(i+1,:));
                num_collisions = num_collisions + 1;
                plot(link1, 'FaceColor','r');
                plot(link2, 'FaceColor','b');
            end
        end
		
	end

end