%CS5335 ex5 opt_c
global T belief_init
T = 20;
belief_init = [2;2;5];
x_init = zeros(3,T);
x_init(3,:) = 5* ones(1,T);
x = fmincon(@cost, x_init, [],[],[],[],[],[],@nonlcon);
%update variances in x(3)
value = x(1,1);
be_v = belief_init(3);
cov = be_v * eye(2);
sigma_w =  0.5 * (5 - value)^2 + 0.1;
noise_w = sigma_w * eye(2);
k = cov * inv(cov + noise_w);
cov_t = (eye(2) - k) * cov;
x(3,1) = cov_t(1,1);
for i = 2:T
    value = x(1,i);
    be_v = x(3, i-1);
    cov = be_v * eye(2);
    sigma_w =  0.5 * (5 - value)^2 + 0.1;
    noise_w = sigma_w * eye(2);
    k = cov * inv(cov + noise_w);
    cov_t = (eye(2) - k) * cov;
    x(3,i) = cov_t(1,1);
end

belief_total = [belief_init x];
disp(belief_total)
%plot(belief_total(1,:), belief_total(2,:), '-o', 'MarkerSize',10*(1:T+1:belief_total(3)))
scatter(belief_total(1,:), belief_total(2,:),100*belief_total(3,:))
hold on;
plot(belief_total(1,:), belief_total(2,:));
function [c,ceq] = nonlcon(x)
global T belief_init
    value = x(1,1);
    be_v = belief_init(3);
    cov = be_v * eye(2);
    sigma_w =  0.5 * (5 - value)^2 + 0.1;
    noise_w = sigma_w * eye(2);
    k = cov * inv(cov + noise_w);
    cov_t = (eye(2) - k) * cov;
    x(3,1) = cov_t(1,1);
    for i = 2:T
        value = x(1,i);
        be_v = x(3, i-1);
        cov = be_v * eye(2);
        sigma_w =  0.5 * (5 - value)^2 + 0.1;
        noise_w = sigma_w * eye(2);
        k = cov * inv(cov + noise_w);
        cov_t = (eye(2) - k) * cov;
        x(3,i) = cov_t(1,1);
    end
    c = x(3,T) - 0.01;
    ceq(1) = x(1,T) - 4;
    ceq(2) = x(2,T);
    ceq = ceq';
end

function p = cost(x)
global T belief_init
    u0 = x(1:2,1) - belief_init(1:2);
    variance = belief_init(3);
    %add variance into cost function
    p = u0(1)^2+u0(2)^2+ variance;
    for i = 1:T-1
        u = x(:,i+1) - x(:,i);
        u_i = u(1)^2 + u(2)^2;
        value = x(1,i);
        cov = variance * eye(2);
        sigma_w =  0.5 * (5 - value)^2 + 0.1;
        noise_w = sigma_w * eye(2);
        k = cov * inv(cov + noise_w);
        cov_t = (eye(2) - k) * cov;
        variance = cov_t(1,1);
        p = p + u_i+variance;
    end
end
