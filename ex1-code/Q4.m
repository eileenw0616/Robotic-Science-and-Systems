% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory. The end effector should trace a circle in the
%                 workspace, as specified by the input argument circle.
%                 (orientation is to be ignored)

function traj = Q4(f, qInit, circle, velocity)
	epsilon = 0.05;
	[~,n] = size(circle); %size give a row vector of nums of rows and cols
    traj = qInit;
	q_init = qInit;
    for i = 1:n-1
		posGoal = circle(:,i+1);
		q = Q3(f, q_init, posGoal, epsilon, velocity);
		[~,m] = size(q);
		q_init = q(m,:);
        q_1 = q(2:m,:);
		traj = vertcat(traj,q_1);
    end

end