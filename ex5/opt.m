%CS5335 ex5 opt
global T x0
T = 20;
x0 = [2;2];
x_goal = [4;0];
x_init = zeros(2,T);
x = fmincon(@cost, x_init, [],[],[],[],[],[],@nonlcon);
xi = [x0 x];
disp(xi)
plot(xi(1,:), xi(2,:))

function [c,ceq] = nonlcon(x)
global T
    c = [];
    ceq(1) = x(1,T) - 4;
    ceq(2) = x(2,T);
    ceq = ceq';
end

function p = cost(x)
global T x0
    u0 = x(:,1) - x0;
    p = u0(1)^2+u0(2)^2;
    for i = 1:T-1
        u = x(:,i+1) - x(:,i);
        u_i = u(1)^2 + u(2)^2;
        p = p + u_i;
    end
end
function p = cost1(x)
global T x0
    u0 = x(:,1) - x0;
    p = 0.5*u0(1)^2+u0(2)*2;
    for i = 1:T-1
        u = x(:,i+1) - x(:,i);
        u_i = 0.5*u(1)^2+ 2*u(2)^2;
        p = p + u_i;
    end
end
function p = cost2(x)
global T x0
    u0 = x(:,1) - x0;
    p = u0(1)^2+u0(2);
    for i = 1:T-1
        u = x(:,i+1) - x(:,i);
        u_i = u(1)^2+ u(2);
        p = p + u_i;
    end
end
function p = cost3(x)
global T x0
    u0 = x(:,1) - x0;
    p = u0(1)+u0(2)^2;
    for i = 1:T-1
        u = x(:,i+1) - x(:,i);
        u_i = u(1)+ u(2)^2;
        p = p + u_i;
    end
end