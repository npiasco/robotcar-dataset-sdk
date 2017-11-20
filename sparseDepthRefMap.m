%%
clear all;
close all;
clc;

addpath matlab/
%%
train_2 = '2014-06-23-15-36-04-light';
train_3 = '2014-06-24-14-09-07';
train_4 = '2014-06-26-08-53-56';
train_5 = 'training/2015-08-28-09-50-22-light';
train_6 = 'training/2015-11-10-14-15-57-light';
train_7 = 'training/2015-05-19-14-06-38-light';
train_8 = 'training/2015-02-10-11-58-05-light';
%local test run_numbers = '2014-06-24-14-09-07-light'
eval_dataset = '2014-06-26-09-31-18-light';
eval_query = '2014-06-24-14-47-45-light';
    
opt_laser = 'ldmrs/';

run_numbers = {train_5, train_6};
for i = 1:length(run_numbers)
	run_number = run_numbers{i};
	root_dir = ['/data/visual_based_localization/Robotcar/' run_number '/'];
    %local test root_dir =['/media/nathan/Data/Robotcar/training/' run_numbers '/']
	timestamps = [root_dir 'timestamps.txt'];
	SaveSparseDepthRefMaps(timestamps, [true true true true], root_dir, true);
end