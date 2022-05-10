%CS 5335 EX3 KF(a)

noise_v = 0.04;
noise_w = 0.01;
u_list = [-0.5 1.2 0.3];
z_list = [-0.7 0.6 0.95];
belief = [0 1];
for i = 1:3
    belief = kf1d(belief, noise_v, noise_w, u_list(i), z_list(i));
end
disp(belief)

function belief = kf1d(belief, noise_v, noise_w, u, z)
    x = belief(1);
    cov = belief(2);
    %prediction
    x_pred = x + u;
    cov_pred = cov + noise_v;
    %innovation
    v = z - x_pred;
    %kalman gain
    k = cov_pred/(cov_pred + noise_w);
    %update
    x_t = x_pred + k*v;
    cov_t = (1 - k)*cov_pred;
    belief = [x_t, cov_t];
end
