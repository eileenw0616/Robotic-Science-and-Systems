% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to plot the robot at

function C1(robot, q)
    % The following code plots the robot in configuration q = [0; 0].
    % You should remove the following code and replace it with code that
    % plots the robot links and pivots at the provided input configuration.
    [link1, link2, pivot1, pivot2] = q2poly(robot, q);
    % Plot the links
    plot(link1, 'FaceColor', 'r');
    plot(link2, 'FaceColor', 'b');
    %plot(polyshape(link1_at0(1,:), link1_at0(2,:)), 'FaceColor', 'r');
    %plot(polyshape(link2_at0(1,:), link2_at0(2,:)), 'FaceColor', 'b');
    % Plot the pivot points
    plot(pivot1(1), pivot1(2), 'k.', 'MarkerSize', 10);
    plot(pivot2(1), pivot2(2), 'k.', 'MarkerSize', 10);
    %plot(origin2_at0(1), origin2_at0(2), 'k.', 'MarkerSize', 10);

end