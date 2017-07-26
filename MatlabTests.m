%%
clear all;
close all;
clc;

addpath matlab/
%%
extrinsics_dir = 'extrinsics/';
models_dir = 'models/';

ins_file = '../2014-05-06-13-09-52/vo/vo.csv';
images_dir = '../2014-05-06-13-09-52/mono_left/';
laser_dir = '../2014-05-06-13-09-52/lms_front/';

images_timestamps_file = '../2014-05-06-13-09-52/mono_left.timestamps';
images_timestamps = dlmread(images_timestamps_file);

laser_timestamps_file = '../2014-05-06-13-09-52/lms_front.timestamps';
laser_timestamps = dlmread(laser_timestamps_file);

%%
ProjectLaserIntoCamera(images_dir, laser_dir, ins_file, models_dir, extrinsics_dir, images_timestamps(1800,1));

%%
BuildPointcloud(laser_dir, ins_file, extrinsics_dir, laser_timestamps(1,1), laser_timestamps(end,1), laser_timestamps(floor(length(laser_timestamps)/2),1));