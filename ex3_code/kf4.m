%CS 5335 EX3 KF(d)

noise_v = [0.04 0;0 0.09];
noise_w = 0.01;
u_list = [-0.5 1.2 0.3; 0.3 -0.6 0.3];
z_list = [0.6 0.4 1.0];
belief = [0 1 0;0 0 1];
for i = 1:3
    belief = kfd(belief, noise_v, noise_w, u_list(:,i), z_list(:,i));
end
disp(belief)

function belief = kfd(belief, noise_v, noise_w, u, z)
    x = belief(:,1);
    cov = belief(:,2:3);
    %prediction
    x_pred = x + u;
    cov_pred = cov + noise_v;
    %innovation
    h = x_pred(1)^2 + x_pred(2)^2;
    v = z - h;
    Hx = [2*x_pred(1) 2*x_pred(2)];
    Hx_t = Hx.';
    %kalman gain
    k = cov_pred*Hx_t*inv(Hx*cov_pred*Hx_t + noise_w);
    %update
    x_t = x_pred + k*v;
    cov_t = cov_pred - k*Hx*cov_pred;
    belief = [x_t, cov_t];
end
