% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    q1 = q(1);
    q2 = q(2);
    Rq1 = rot2(q1);
    Rq2 = rot2(q2);
    pivot1 = robot.pivot1;
    link1 = Rq1 * robot.link1 + pivot1;
    pivot2 = Rq1 * robot.pivot2 + pivot1;
    link2 = Rq2 * Rq1 * robot.link2 + pivot2;
    poly1 = polyshape(link1');
    poly2 = polyshape(link2');



end