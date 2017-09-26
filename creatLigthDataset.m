%%
clear all;
close all;
clc;

addpath matlab/
%%
root_dir = ['../' ];
train_2 = '2014-06-23-15-36-04';
train_3 = '2014-06-24-14-09-07';
train_4 = '2014-06-26-08-53-56';
eval_dataset = '2014-06-26-09-31-18';
eval_query = '2014-06-24-14-47-45';
run_number = eval_query;


timestamps = [root_dir run_number '-light/timestamps.txt'];
im_ts = dlmread(timestamps);
n_im = length(im_ts);

mkdir([root_dir run_number '-light/mono_rear']);
mkdir([root_dir run_number '-light/mono_left']);
mkdir([root_dir run_number '-light/mono_right']);
mkdir([root_dir run_number '-light/stereo/centre']);

cameras_dir = {'mono_left/', 'mono_right/', 'stereo/centre/'};
models = cell(size(cameras_dir));

for i=1:length(cameras_dir)
    [~, ~, ~, ~, ~, models{i}] = ...
        ReadCameraModel([cameras_dir{i}],  'models/');
end

for i=1:n_im
    for j=1:length(cameras_dir)
        image = LoadImage([root_dir run_number '/' cameras_dir{j}], im_ts(i,j),models{j});
        imwrite(image, [root_dir run_number '-light/' cameras_dir{j} num2str(im_ts(i,j)) '.jpg'], 'jpg');
    end
end