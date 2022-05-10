%CS5335 ex5 v1
clf
im0 = iread('imgs/rvc2_cover.png','mono','double');
im1 = iread('imgs/im1.jpeg','mono','double');
im2 = iread('imgs/im2.jpeg','mono','double');
sf0 = isurf(im0);
sf1 = isurf(im1);
sf2 = isurf(im2);
[m0, corre0] = match(sf0,sf1);
[h0, r0] = m0.ransac(@homography,4);
[m1, corre1] = match(sf1,sf2);
[h1, r1] = m1.ransac(@homography,4);
%sf1.plot_scale('g')
%idisp({im0,im1},'dark')
%m0.plot('w')
f1 = figure(1);
homwarp(h0, im0, 'full');
f2 = figure(2);
for i = 1:3
    m1 = m1.outlier;
    [h1, r1] = m1.ransac(@homography,4);
    homwarp(h1, im1, 'full');
end
R0 = h0(1:2,1:2);
t0 = h0(1:2,3);
R1 = h1(1:2,1:2);
t1 = h1(1:2,3);
disp(h0);
disp(h1);
R0 = h0