%CS5335 ex5 v3_d
function [center,axis,radius,length] = fitCylinder(list)
threshold = 0.001;
lastInliers = 0;
num_iterations = 2000;
for k = 1:num_iterations
    inliers = 0; 
    % Random points
    [~,num] = size(list);
    rand_index = randi(num);
    rand_point = list(:,rand_index);
    while isnan(rand_point)
        rand_index = randi(num);
        rand_point = list(:,rand_index);
    end
    point1 = rand_point;
    idx1 = rand_index;
    rand_index = randi(num);
    rand_point = list(:,rand_index);
    while isnan(rand_point) | (rand_index == idx1)
        rand_index = randi(num);
        rand_point = list(:,rand_index);
    end
    point2 = rand_point;
    idx2 = rand_index;
    normal1 = surfaceNorm(point1,r,list);
    normal2 = surfaceNorm(point2,r,list);
     % Find cylinder axis
    ax = cross(normal1,normal2);
    rad = 0.05 + rand(1)*(0.10 - 0.05); 
    % Sphere is between radius 0.10m and 0.05m
    centroid = point1 + rad*normal1';

        
    if inliers > lastInliers
        lastInliers = inliers;
        center = centroid';
        radius = rad;
        axis = ax;
    end
end
end
function [sur_norm] = surfaceNorm(point, r, list)
    neighbors = [];
    [~,num] = size(list);
    for i = 1:num
            test = list(:,i);
            test = double(test);
            check = find(isnan(test),1);
            %check point is nan or not
            if isempty(check)
                dist = sqrt((point(1)-test(1))^2+(point(2)-test(2))^2+(point(3)-test(3))^2);
                if dist > r
                    continue;
                elseif isequal(test,point) == 0
                    neighbors = [neighbors test];
                end
            else
                continue;
            end
    end
    vari = 0;
    [~,c] = size(neighbors);
    for k = 1:c
        nk = neighbors(:,k);
        diff = nk - point;
        diff_k = diff' * diff;
        vari = vari + diff_k;
    end
    vari_m = vari * ones(3,3);
    [vec,val] = eig(vari_m);
    [~,minind] = min([val(1,1),val(2,2),val(3,3)]);
    sur_norm = vec(:,minind)';
end