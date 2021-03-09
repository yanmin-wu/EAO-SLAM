clearvars
close all
clc

result_dir = 'result_euroc_v201';

downsample_rate = 0.1;

ptCloud = pcread(strcat(result_dir,'/data.ply'));
ptCloudOut = pcdownsample(ptCloud,'random',downsample_rate);
figure
pcshow(ptCloudOut);
title('Ground Truth');

semidense = pcread(strcat(result_dir,'/semi_pointcloud.obj.ply'));
semidenseOut = pcdownsample(semidense,'random',downsample_rate);
figure
pcshow(semidenseOut);
title('Semidense Pointcloud');

pcwrite(ptCloudOut, strcat(result_dir,'/data_down.ply'));
pcwrite(semidenseOut,strcat(result_dir,'/semi_down.ply'));



