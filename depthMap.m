%%
clear all;
close all;
clc;

addpath matlab/
%%
train_2 = '2014-06-23-15-36-04';
train_3 = '2014-06-24-14-09-07';
train_4 = '2014-06-26-08-53-56';

run_number = train_2
root_dir = ['../training/' run_number];
timestamps = ['../training/' run_number '/TrainDataset2/timestamps.txt'];
SaveDepthMaps( timestamps, [true true true], root_dir, true, true);
