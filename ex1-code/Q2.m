% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2(f, qInit, posGoal)
    q = qInit;
    R = [1 0 0;0 1 0;0 0 1];
    T = [R posGoal;0 0 0 1];
    alpha = 0.5;

    for i = 1:50
        fk = f.fkine(q);
        j = f.jacob0(q);
        jinv = pinv(j);
        dx = tr2delta(fk,T);
        dq = alpha * jinv * dx;
        q = q + dq';
    end

end