clearvars
close all
clc

result_dir = 'result_euroc_v201';

init_scale = 2.500;

ptCloudOut = pcread(strcat(result_dir,'/data_down.ply'));
semidenseOut = pcread(strcat(result_dir,'/semi_down.ply'));

fun = @(scale)scaling(ptCloudOut,semidenseOut,scale);

scaleMin = fminsearch(fun, init_scale);

xyzpoints = semidenseOut.Location * scaleMin;

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

pcwrite(semidenseScale,strcat(result_dir,'/semi_init.ply'));

[tform3,semiTransform,rmse] = pcregrigid(semidenseScale,ptCloudOut,'Extrapolate',true);

pcwrite(semiTransform,strcat(result_dir,'/semi_transform.ply'));

semiTest = pctransform(semidenseScale, tform3);
pcwrite(semiTest,strcat(result_dir,'/semi_test.ply'));

save(strcat(result_dir,'/tforms'),'scaleMin','tform1','tform2','tform3');


