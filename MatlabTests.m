%%
clear all;
close all;
clc;

addpath matlab/
%%
run_number = '2014-06-24-14-47-45';
extrinsics_dir = 'extrinsics/';
models_dir = 'models/';

vo_file = ['../' run_number '/vo/vo.csv'];
ins_file = ['../' run_number '/gps/ins.csv'];

images_dir = ['../' run_number '/mono_left/'];

laser_dir = ['../' run_number '/lms_front/'];

images_timestamps_file = ['../' run_number '/mono_left.timestamps'];
images_timestamps = dlmread(images_timestamps_file);

laser_timestamps_file = ['../' run_number '/lms_front.timestamps'];
laser_timestamps = dlmread(laser_timestamps_file);

gps_file = ['../' run_number '/gps/gps.csv'];

%%
ProjectLaserIntoCamera(images_dir, laser_dir, vo_file, models_dir, extrinsics_dir, images_timestamps(3800,1));

%%
BuildPointcloud(laser_dir, ins_file, extrinsics_dir, laser_timestamps(1,1), laser_timestamps(end,1), laser_timestamps(floor(length(laser_timestamps)/2),1));

%%
PlayImages(images_dir, models_dir);

%%
PlotGPS(gps_file);
PlotVO(vo_file);
PlotINS(ins_file);

CompareVOtoINS(ins_file, vo_file)