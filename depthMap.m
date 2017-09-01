%%
clear all;
close all;
clc;

addpath matlab/
%%
train_2 = '2014-06-23-15-36-04-light';
train_3 = '2014-06-24-14-09-07';
train_4 = '2014-06-26-08-53-56';
    
opt_laser = 'ldmrs/';

run_number = train_2;
root_dir = ['../training/' run_number '/'];
timestamps = ['../training/' run_number '/timestamps.txt'];
SaveDepthRefMaps( timestamps, [true true true true], root_dir, true);

%SaveDepthMaps( timestamps, [true true true true], root_dir, true);
%SaveDepthMaps( timestamps, [true true true true], root_dir, true, opt_laser);


%im = dlmread(timestamps);
%[im, rm, fm, outMask, maxDep, maxRef] = PrepareDataToDepthMapCreation([root_dir 'stereo/centre'], [root_dir 'lms_front/'], ...
%    [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', im(200,4));
