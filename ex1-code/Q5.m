% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First seven joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)

function q = Q5(f1, f2, qInit, f1Target, f2Target)
    q = qInit;
    alpha = 0.5;
    R = [1 0 0; 0 1 0; 0 0 1];
    t1 = [R f1Target; 0 0 0 1];
    t2 = [R f2Target; 0 0 0 1];

    for i = 1:50
        q1 = [q(:, 1:7) q(:, 8: 9)];
        q2 = [q(:,1:7),q(:,10:11)];
        dx1 = tr2delta(f1.fkine(q1),t1);
        dx2 = tr2delta(f2.fkine(q2),t2);
        dx = [dx1; dx2];
        j1 = f1.jacob0(q1);
        j1 = [j1(:,1:7), j1(:,8:9),zeros(6,2)];
        j2 = f2.jacob0(q2);
        j2 = [j2(:,1:7), zeros(6,2),j2(:,8:9)];
        jinv = pinv([j1;j2]);
        dq = alpha*jinv*dx;
        q = q + dq';
    end
end