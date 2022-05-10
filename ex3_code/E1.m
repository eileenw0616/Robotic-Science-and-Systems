% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks) for localization
% Output: x_est -> 1xT cell array containing the vehicle state mean
%                  for T time steps (i.e., x_est{t} is a 3x1 vector)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a 3x3 matrix)

function [x_est, P_est] = E1(odo, zind, z, V, W, x0, P0, map)
    T = length(zind);
    x = x0;
    p = P0;
    x_est = cell(1,T);
    P_est = cell(1,T);
    for t = 1:T
        curr_odo = odo(:,t);
        % prediction
        Fx = [1 0 -curr_odo(1)*sin(x(3));0 1 curr_odo(1)*cos(x(3));0 0 1];
        Fv = [cos(x(3)) 0;sin(x(3)) 0;0 1];
        x_pred = [x(1)+curr_odo(1)*cos(x(3));x(2)+curr_odo(1)*sin(x(3));x(3)+curr_odo(2)];
        p_pred = Fx * p * Fx.' + Fv * V * Fv.';
        if zind(t) ~= 0
            landmark = map.landmark(zind(t));
            % innovation
            r = sqrt((landmark(2) - x_pred(2))^2 + (landmark(1) - x_pred(1))^2);
            h = [r;angdiff(atan2(landmark(2)-x_pred(2),landmark(1)-x_pred(1)),x_pred(3))];
            inno = z{t} - h;
            Hx = [-(landmark(1)-x_pred(1))/r -(landmark(2)-x_pred(2))/r 0;(landmark(2)-x_pred(2))/(r^2) -(landmark(1)-x_pred(1))/(r^2) -1];
            Hw = eye(2);
            s = Hx * p_pred * Hx.' + Hw * W * Hw.';
            k = p_pred * Hx.' * inv(s);
            x = x_pred + k * inno;
            p = p_pred - k * Hx * p_pred;
        else
            x = x_pred;
            p = p_pred;
        end
        x_est{t} = x;
        P_est{t} = p;
    end
end