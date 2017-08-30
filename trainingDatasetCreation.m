%%
clear all;
close all;
clc;

addpath matlab/
%%
train_1 = '2014-05-06-13-09-52'; % BAD
train_2 = '2014-06-23-15-36-04';
train_3 = '2014-06-24-14-09-07';
train_4 = '2014-06-26-08-53-56';

extrinsics_dir = 'extrinsics/';
models_dir = 'models/';

%%

run_number = train_4;

ref_gps_file =  ['../training/' train_1 '/gps/gps.csv'];
vo_file = ['../training/' run_number '/vo/vo.csv'];
ins_file = ['../training/' run_number '/gps/ins.csv'];

images_dir = ['../training/' run_number '/stereo/centre/'];

laser_dir = ['../training/' run_number '/lms_front/'];

gps_file = ['../training/' run_number '/gps/gps.csv'];


%%
CompareVOtoINS(ins_file, vo_file, gps_file)
%%
images_dir = ['../training/' run_number '/'];
CreatDataset( ins_file, ref_gps_file, images_dir, models_dir, 2, [true true true true], true, 'TrainDataset4v2'); % Dataset