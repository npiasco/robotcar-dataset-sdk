%%
clear all;
close all;
clc;

addpath matlab/
%%
%run_number = '2014-06-24-14-47-45'; % Query with front + left + right
%run_number = '2014-12-05-15-42-07';
%run_number = '2014-06-26-09-31-18'; % Robotcar_D1
run_number = '2014-07-14-14-49-50' % Full Oxford
extrinsics_dir = 'extrinsics/';
models_dir = 'models/';

%ref_gps_file = '../2014-12-05-15-42-07/gps/gps.csv';
ref_gps_file =  ['../' run_number '/gps/gps.csv'];
vo_file = ['../' run_number '/vo/vo.csv'];
ins_file = ['../' run_number '/gps/ins.csv'];

images_dir = ['../' run_number '/stereo/centre/'];

laser_dir = ['../' run_number '/lms_front/'];

gps_file = ['../' run_number '/gps/gps.csv'];

%%

images_timestamps_file = ['../' run_number '/stereo.timestamps'];
images_timestamps = dlmread(images_timestamps_file);

laser_timestamps_file = ['../' run_number '/lms_front.timestamps'];
laser_timestamps = dlmread(laser_timestamps_file);

ProjectLaserIntoCamera(images_dir, laser_dir, ins_file, models_dir, extrinsics_dir, images_timestamps(3900,1));
BuildPointcloud(laser_dir, ins_file, extrinsics_dir, laser_timestamps(1,1), laser_timestamps(end,1), laser_timestamps(floor(length(laser_timestamps)/2),1));

%%
PlayImages(images_dir, models_dir);

%%
PlotGPS(gps_file, true);
PlotVO(vo_file, true);
[X Y] = PlotINS(ins_file, true);

CompareVOtoINS(ins_file, vo_file, gps_file)
%%

images_dir = ['../' run_number '/'];
CreatDataset( ins_file, ref_gps_file, images_dir, models_dir, 15, [true true true true], true, 'Dataset', 0.85); % Dataset
%CreatDataset( ins_file, ref_gps_file, images_dir, models_dir, 30, [true false true true], true, 'Query'); % Query