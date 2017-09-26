%%
clear all;
close all;
clc;

addpath matlab/
%%
train_2 = '2014-06-23-15-36-04-light';
train_3 = '2014-06-24-14-09-07';
train_4 = '2014-06-26-08-53-56';
eval_dataset = '2014-06-26-09-31-18-light';
eval_query = '2014-06-24-14-47-45-light';
    
opt_laser = 'ldmrs/';

run_number = 'Robotcar_D1/Dataset';
root_dir = ['/data/visual_based_localization/Robotcar/' run_number '/'];
timestamps = ['/data/visual_based_localization/Robotcar/' run_number '/timestamps.txt'];

SaveDepthMaps( timestamps, [true true true true], root_dir, true);

run_number = 'Robotcar_D1/Query';
root_dir = ['/data/visual_based_localization/Robotcar/' run_number '/'];
timestamps = ['/data/visual_based_localization/Robotcar/' run_number '/timestamps.txt'];

SaveDepthMaps( timestamps, [true true true true], root_dir, true);
