%CS 5335 EX3 KF(b)

noise_v = [0.04 0;0 0.09];
noise_w = [0.01 0;0 0.02];
u_list = [-0.5 1.2 0.3;0.3 -0.6 0.3];
z_list = [-0.7 0.6 0.95;0.3 0.0 0.15];
belief = [0 1 0;0 0 1];
for i = 1:3
    belief = kf2d(belief, noise_v, noise_w, u_list(:,i), z_list(:,i));
end
disp(belief)

function belief = kf2d(belief, noise_v, noise_w, u, z)
    x = belief(:,1);
    cov = belief(:,2:3);
    %prediction
    x_pred = x + u;
    cov_pred = cov + noise_v;
    %innovation
    v = z - x_pred;
    %kalman gain
    k = cov_pred*inv(cov_pred + noise_w);
    %update
    x_t = x_pred + k*v;
    cov_t = (eye(2) - k)*cov_pred;
    belief = [x_t, cov_t];
end
