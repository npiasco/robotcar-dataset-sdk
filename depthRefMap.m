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
train_night = 'training/2014-12-16-18-44-24-light';
train_snow = 'training/2015-02-03-08-45-10-light';

eval_dataset = '2014-06-26-09-31-18-light';
eval_query = '2014-06-24-14-47-45-light';
eval_night = 'training/2014-12-16-18-44-24-light-query';
eval_snow = 'training/2015-02-03-08-45-10-light-query';
    
opt_laser = 'ldmrs/';

%run_numbers = {train_5, train_6};
%run_numbers = {train_5, train_6, train_7};
%run_numbers = {train_night};
run_numbers = {train_5};
for i = 1:length(run_numbers)
	run_number = run_numbers{i}
	root_dir = ['/data/visual_based_localization/Robotcar/' run_number '/'];
	timestamps = ['/data/visual_based_localization/Robotcar/' run_number '/timestamps.txt'];
	%timestamps = ['/data/visual_based_localization/Robotcar/Robotcar_D1/NightQuery/timestamps.txt'];
	%SaveDepthRefMaps(timestamps, [true true true true], root_dir, true);
	TrueDepthRefMaps(timestamps, [true true true true], root_dir, true);
end

%SaveDepthMaps( timestamps, [true true true true], root_dir, true);
%SaveDepthMaps( timestamps, [true true true true], root_dir, true, opt_laser);


%im = dlmread(timestamps);
%[im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'mono_rear'], [root_dir 'lms_front/'], ...
%    [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', im(1,2));
