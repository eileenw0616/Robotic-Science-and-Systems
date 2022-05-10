%CS5335 ex5 v2
load bunny.mat;
M = bunny;
R_act = rand([1,3]);
t_act = rand([1,3]);
T_rand = SE3(t_act)*SE3.rpy(R_act);
disp('Random T: ');
disp(T_rand);
D = T_rand * M;
%[T,d] = icp(M,D,'plot')
[T, error] = v2icp(M, D);
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
        end
        [U, S, V] = svd(W);
        R = V*U';
        t = (D_mean - M_mean)';
        %update
        T = T*rt2tr(R,t);
        error = norm(dist);

    end
end
