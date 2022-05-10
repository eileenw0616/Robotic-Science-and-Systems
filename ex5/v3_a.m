%CS5335 ex5 v3_a
load("ptcloud.mat");
rgb = ptcloud_rgb;
xyz = ptcloud_xyz;
r = 0.3;
%randomly choose a point
rand_point = xyz(randi(480),randi(640),:);
while isnan(rand_point)
    rand_point = xyz(randi(480),randi(640),:);
end
point = squeeze(rand_point);
point = double(point);
disp('Random point: ');
disp(point);
[sur_norm] = surfaceNorm(point, r, xyz);
disp('Surface Normal: ');
disp(sur_norm);

function [sur_norm] = surfaceNorm(point, r, xyz)
    neighbors = [];
    for i = 1:480
        for j = 1:640
            test = xyz(i,j,:);
            test = double(squeeze(test));
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
