%CS5335 ex5 v2_c
%Remove 40 points randomly and add noise, as a result error increased a lot
load bunny.mat;
M = bunny;
R_act = rand([1,3]);
t_act = rand([1,3]);
T_rand = SE3(t_act)*SE3.rpy(R_act);
D = T_rand * M;
%randomly remove 40 points
D(:,randi(numcols(D),40,1)) = [];
%add noise
D = D + 0.01*randn(size(D));
%[T,d] = icp(M,D,'plot')
[T, error] = v2icp(M, D);
disp('Random T: ');
disp(T_rand);
disp('ICP result: ');
disp(T);
disp('Error: ');
disp(error);
function [T, error] = v2icp(M, D)
    iter = 100;
    M_mean = mean(M');
    D_mean = mean(D');
    t = D_mean - M_mean;
    T = transl(t);
    for i=1:iter
        %update
        Mi = homtrans(T,M);
        %closest point pairs
        [corre, dist] = closest(D, Mi);
        %center the 2 point clouds
        M_mean = mean(Mi(:,corre)');
        D_mean = mean(D');
        %calculate part
        W = zeros(3,3);
        for i=1:numcols(D)
            ci = corre(i);
            W = W + (Mi(:,ci) - M_mean')*(D(:,i) - D_mean')';
            %W = W + (Mk(:,ic) - Mbar') * (Dp(:,i) - Dbar')';
        end
        [U, S, V] = svd(W);
        R = V*U';
        t = (D_mean - M_mean)';
        T = T*rt2tr(R,t);
        error = norm(dist);clc
    end
end