% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the known initial vehicle state
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (2M)x(2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first two
%                    rows of x_est and P_est correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E2(odo, zind, z, W, x0)
    T = length(zind);
    v = x0;
    x = [];
    p = [];
    m = 0;
    indices = [];
    x_est = cell(1,T);
    P_est = cell(1,T);
    for t = 1:T
        curr_odo = odo(:,t);
        v = [v(1)+curr_odo(1)*cos(v(3));v(2)+curr_odo(1)*sin(v(3));v(3)+curr_odo(2)];
        z_t = zind(t);
        if z_t ~= 0
            if ismember(z_t, indices)
                i = find(indices==z_t);
                landmark = [x(i*2-1) x(i*2)];
                r = sqrt((landmark(2) - v(2))^2 + (landmark(1) - v(1))^2);
                h = [r;angdiff(atan2(landmark(2)-v(2),landmark(1)-v(1)),v(3))];
                Hpi = [(landmark(1)-v(1))/r (landmark(2)-v(2))/r; -(landmark(2)-v(2))/(r^2) (landmark(1)-v(1))/(r^2)];
                Hx = zeros(2, 2*m);
                Hx(1:2, i*2-1:i*2) = Hpi;
                Hw = eye(2);
                inno = z{t} - h;
                s = Hx * p * Hx.' + Hw * W * Hw.';
                k = p * Hx.' * inv(s);
                x = x + k * inno;
                p = p - k * Hx * p;
            else
                indices = [indices;z_t];
                m = m + 1;
                x = [x;v(1)+z{t}(1)*cos(v(3)+z{t}(2));v(2)+z{t}(1)*sin(v(3)+z{t}(2))];
                Yz = eye(2*m);
                Yz(2*m-1:2*m, 2*m-1:2*m) = [cos(v(3)+z{t}(2)) -z{t}(1)*sin(v(3)+z{t}(2));sin(v(3)+z{t}(2)) z{t}(1)*cos(v(3)+z{t}(2))];
                p = Yz * blkdiag(p,W) * Yz.';
            end
        end
        x_est{t} = x;
        P_est{t} = p;
    end
end