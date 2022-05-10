% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
    [~,n] = size(cspace);
    padded_cspace = zeros(n, n);
    neighbors = [-1 0 1 1 1 0 -1 -1; 1 1 1 0 -1 -1 -1 0];
    for i = 2:n-1
        for j = 2:n-1
            if cspace(i,j) == 1
                for k = 1:8
                    newx = i + neighbors(1, k);
                    newy = j + neighbors(2, k);
                    if cspace(newx, newy) == 0
                        padded_cspace(newx, newy) = 1;
                    end
                end
            end
        end
    end
end