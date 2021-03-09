function rmse = scaling(ptCloud, semidense, scale)

xyzpoints = semidense.Location * scale;

semidenseScale = pointCloud(xyzpoints);

% init transform
% around x axis
A = [1 0 0 0; ...
    0 cos(pi/2) -sin(pi/2) 0; ...
    0 sin(pi/2) cos(pi/2)  0; ...
            0         0  0 1];
tform1 = affine3d(A);
semidenseScale = pctransform(semidenseScale,tform1);

% around z axis
B = [cos(pi/2) sin(pi/2) 0 0; ...
    -sin(pi/2) cos(pi/2) 0 0; ...
            0         0    1 0; ...
            0         0    0 1];
tform2 = affine3d(B);
semidenseScale = pctransform(semidenseScale,tform2);


[tform,semiTransform,rmse] = pcregrigid(semidenseScale,ptCloud,'Extrapolate',true);

display(scale)
display(rmse)
