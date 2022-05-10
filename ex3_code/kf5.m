%CS 5335 EX3 KF(e)

noise_v = [0.01 0;0 0.01];
u_list = [1.0 2.0 -5.0;-1.0 -1.0 0.0];
z_list = [3.5 5.3 -2.0;-1.0 -0.5 0.0];
belief = [2 5 0;2 0 5];
for i = 1:3
    value = belief(1,1);
    sigma_w = 0.5 * (5 - value)^2 + 0.01;
    noise_w = [sigma_w 0; 0 sigma_w];
    belief = kfld(belief, noise_v, noise_w, u_list(:,i), z_list(:,i));
end
disp(belief)

function belief = kfld(belief, noise_v, noise_w, u, z)
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