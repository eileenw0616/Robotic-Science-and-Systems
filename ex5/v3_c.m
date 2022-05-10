%CS5335 ex5 v3_c find sphere
%Sample a point, sample a radius, compute the surface normal at the point,follow the normal direction for a distance equal to the sampled radius, we arrive at the center of the sphere
%candidate.
%Cost about 5 mins.
load("ptcloud.mat");
rgb = ptcloud_rgb;
xyz = ptcloud_xyz;
l1 = xyz(:,:,1);
l1 = reshape(l1, [480*640, 1]);
l2 = xyz(:,:,2);
l2 = reshape(l2, [480*640, 1]);
l3 = xyz(:,:,3);
l3 = reshape(l3, [480*640, 1]);
list = [l1 l2 l3]';
[center,radius] = fitSphere(list);
    disp('Center: ')
    disp(center);
    disp('Radius: ')
    disp(radius); 
function [center,radius] = fitSphere(list)
threshold = 0.001;
lastInliers = 0;
num_iterations = 2000;
r = 0.1;
for j = 1:num_iterations
    inliers = 0;
    [~,num] = size(list);
    rand_index = randi(num);
    rand_point = list(:,rand_index);
    while isnan(rand_point)
        rand_index = randi(num);
        rand_point = list(:,rand_index);
    end
    normal = surfaceNorm(rand_point,r,list);
    % Sphere is between radius 0.10m and 0.05m
    rad = 0.05 + rand(1)*(0.1 - 0.05); 
    centroid = rand_point + rad*normal;
    for i = 1:num
        p = list(:,i);
        %distance between points and center
        dist = sqrt((p(1)-centroid(1))^2+(p(2)-centroid(2))^2+(p(3)-centroid(3))^2);
        dist_i = dist - rad;
        if dist_i < threshold
            inliers = inliers + 1;
        end
    end
    if inliers > lastInliers
       lastInliers = inliers;
       center = centroid';
       radius = rad;
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
    sur_norm = vec(:,minind);
end