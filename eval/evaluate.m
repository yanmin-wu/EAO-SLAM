clearvars
close all
clc

% evaluate line segment result of euroc vicon dataset
result_dir = 'result_euroc_v201';

load(strcat(result_dir,'/tforms.mat'))

data = pcread(strcat(result_dir,'/data.ply'));

line3d_load = pcread(strcat(result_dir,'/Line3D++__W_1500__N_10__sigmaP_2.5__sigmaA_10__epiOverlap_0.25__kNN_10__OPTIMIZED__vis_3.obj.ply'));
long_load = pcread(strcat(result_dir,'/line_segments_edlines.obj.ply'));
short_load = pcread(strcat(result_dir,'/line_segments.obj.ply'));
short_c_load = pcread(strcat(result_dir,'/line_segments_clustered_incr.obj.ply'));

% scaling
line3d_points = line3d_load.Location * scaleMin;
line3d = pointCloud(line3d_points);

long_points = long_load.Location * scaleMin;
long = pointCloud(long_points);

short_points = short_load.Location * scaleMin;
short = pointCloud(short_points);

short_c_points = short_c_load.Location * scaleMin;
short_c = pointCloud(short_c_points);

% apply transform
line3d = pctransform(line3d, tform1);
line3d = pctransform(line3d, tform2);
line3d = pctransform(line3d, tform3);

long = pctransform(long, tform1);
long = pctransform(long, tform2);
long = pctransform(long, tform3);

short = pctransform(short, tform1);
short = pctransform(short, tform2);
short = pctransform(short, tform3);

short_c = pctransform(short_c, tform1);
short_c = pctransform(short_c, tform2);
short_c = pctransform(short_c, tform3);

% compute distance to closest ground truth point: brute force 
line3d_dist = euclideanDistanceTwoPointClouds(data.Location,line3d.Location);
long_dist = euclideanDistanceTwoPointClouds(data.Location,long.Location);
short_dist = euclideanDistanceTwoPointClouds(data.Location,short.Location);
short_c_dist = euclideanDistanceTwoPointClouds(data.Location,short_c.Location);


fprintf('avgDist line3d: %f\n',mean(line3d_dist));
fprintf('avgDist long: %f\n',mean(long_dist));
fprintf('avgDist short: %f\n',mean(short_dist));
fprintf('avgDist short_c: %f\n',mean(short_c_dist));


