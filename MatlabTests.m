%%
clear all;
close all;
clc;

addpath matlab/
%%
%run_number = '2014-06-24-14-47-45';
run_number = '2014-12-05-15-42-07';
extrinsics_dir = 'extrinsics/';
models_dir = 'models/';

ref_gps_file = '../2014-12-05-15-42-07/gps/gps.csv';

vo_file = ['../' run_number '/vo/vo.csv'];
ins_file = ['../' run_number '/gps/ins.csv'];

images_dir = ['../' run_number '/stereo/centre/'];

laser_dir = ['../' run_number '/lms_front/'];

images_timestamps_file = ['../' run_number '/stereo.timestamps'];
images_timestamps = dlmread(images_timestamps_file);

laser_timestamps_file = ['../' run_number '/lms_front.timestamps'];
laser_timestamps = dlmread(laser_timestamps_file);

gps_file = ['../' run_number '/gps/gps.csv'];

%%
ProjectLaserIntoCamera(images_dir, laser_dir, ins_file, models_dir, extrinsics_dir, images_timestamps(3900,1));

%%
BuildPointcloud(laser_dir, ins_file, extrinsics_dir, laser_timestamps(1,1), laser_timestamps(end,1), laser_timestamps(floor(length(laser_timestamps)/2),1));

%%
PlayImages(images_dir, models_dir);

%%
PlotGPS(gps_file);
PlotVO(vo_file);
PlotINS(ins_file);

CompareVOtoINS(ins_file, vo_file, gps_file)
%%

images_dir = ['../' run_number '/'];
CreatDataset( ins_file, ref_gps_file, images_dir, models_dir, 30, [false false false true], true, 'Test');

%%
[X, Y] = PlotINS(ins_file);
T1 = [X Y];
[X, Y] = PlotGPS(ref_gps_file);
T2 = [X Y];
T3 = AlignTrajectories( T1, T2 );