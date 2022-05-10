%CS5335 ex5 v3_b identify the planes
%A single call of the function fitPlane will cost a few minutes, so when
%using the while loop(while t < 3) will take a while.
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
t = 0;
normals = [];
while t < 3
    [center, normal] = fitPlane(list);
    if t == 0
        normals = normal;
        t = t + 1;
    elseif t == 1
        if normal == normals(1,:)
            continue;
        else
            normals = [normals;normal];
            t = t + 1;
        end
    elseif t == 2
        if normal == normals(1,:)|normal == normals(2,:)
            continue;
        else
            normals = [normals;normal];
            t = t + 1;
        end        
    end
    disp('Center: ')
    disp(center);
    disp('Normal: ')
    disp(normal);
end
function[center, normal] = fitPlane(list)
r = 0.1;
lastInliers = 0;
threshold = 0.01;
num_iterations = 1000;
for j = 1:num_iterations
%Randomly choose 3 points
    points = zeros(3,3);
    [~,num] = size(list);
    while (points(:,1) == points(:,2)) | (points(:,1) == points(:,3)) | (points(:,2) == points(:,3))
            rand_index = randi(num);
            rand_point = list(:,rand_index);
            while isnan(rand_point)
                rand_index = randi(num);
                rand_point = list(:,rand_index);
            end
            points(:,1) = double(rand_point);
            idx1 = rand_index;
            rand_index = randi(num);
            rand_point = list(:,rand_index);
            while isnan(rand_point)
                rand_index = randi(num);
                rand_point = list(:,rand_index);
            end
            points(:,2) = double(rand_point);
            idx2 = rand_index;
            rand_index = randi(num);
            rand_point = list(:,rand_index);
            while isnan(rand_point)
                rand_index = randi(num);
                rand_point = list(:,rand_index);
            end
            points(:,3) = double(rand_point);
            idx3 = rand_index;
    end
    inliers = 0;
    centroid = mean(points')';
    norm = surfaceNorm(centroid,r,list);
    idx = [idx1 idx2 idx3];
    for j=1:num
        distance = abs(norm*(list(:,j)-centroid));
        if distance < threshold
            inliers = inliers+1;
            idx = [idx j];
        end
    end

    if inliers > lastInliers
        lastInliers = inliers;
        center = centroid;
        normal = norm;
        l = length(idx);
        for k=1:l
            list(:,k) = [NaN;NaN;NaN];
        end
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