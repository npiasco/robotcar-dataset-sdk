%%
clear all;
close all;
clc;

addpath matlab/
%%
extrinsics_dir = 'extrinsics/';
models_dir = 'models/';

%%

%run_number = ['../../robotcar_dw/' '11/2015-11-10-14-15-57'];
%run_number = ['../../robotcar_dw/' '08/2015-08-28-09-50-22'];
%run_number = ['../../robotcar_dw/' '02/2015-02-27-18-09-02'];
%run_number = ['../../robotcar_dw/' '05/2015-05-19-14-06-38'];
run_number = ['../../robotcar_dw/' '02_day/2015-02-10-11-58-05'];


ref_gps_file =  ['../../robotcar_dw/' '11/2015-11-10-14-15-57-light' '/gps/gps.csv'];
vo_file = [run_number '/vo/vo.csv'];
ins_file = [run_number '/gps/ins.csv'];

images_dir = [run_number '/stereo/centre/'];

laser_dir = [run_number '/lms_front/'];

gps_file = [run_number '/gps/gps.csv'];


%%
CompareVOtoINS(ins_file, vo_file, gps_file)
%%
images_dir = [run_number '/'];
step = 5; % metre
crop_path = 0.57; % percent
CreatDataset(gps_file, ref_gps_file, images_dir, models_dir, step, [true true true true], true, 'TrainDataset_02_10_15', crop_path); % Dataset