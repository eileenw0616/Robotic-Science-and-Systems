% input: f -> an 9-joint robot encoded as a SerialLink class
%        position -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q1(f, position)
    R = [1 0 0;0 1 0;0 0 1];
    T = [R position;0 0 0 1];
    m = [1,1,1,0,0,0];
    q = f.ikine(T,'mask',m);

end