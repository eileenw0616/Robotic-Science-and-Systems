%CS 5335 EX3 KF(c)

noise_v = [0.02 0;0 0.03];
noise_w = 0.01;
u_list = [-0.2 0.0 0.1];
z_list = [0.4 0.9 0.8];
belief = [0 1 0;0.5 0 2];
F = [1 1;0 1];
G = [0;1];
H = [1 0];

for i = 1:3
    belief = kf1m(belief, noise_v, noise_w, u_list(i), z_list(i), F, G, H);
end
disp(belief)

function belief = kf1m(belief, noise_v, noise_w, u, z, F, G, H)
    x = belief(:,1);
    cov = belief(:,2:3);
    %prediction 
    % x_t+1 = x_t + v_t + pos_noise m_t+1 = m_t + u_t + vel_noise
    x_pred = F*x + G*u;
    F_t = F.';
    cov_pred = F*cov*F_t + noise_v;
    %innovation
    v = z - (H*x_pred);
    %kalman gain
    H_t = H.';
    k = cov_pred*H_t*inv(H*cov_pred*H_t + noise_w);
    %update
    x_t = x_pred + k*v;
    cov_t = (eye(2) - k*H)*cov_pred;
    belief = [x_t, cov_t];
end