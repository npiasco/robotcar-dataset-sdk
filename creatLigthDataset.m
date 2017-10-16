%%
clear all;
close all;
clc;

addpath matlab/
%%
root_dir = [''];
%run_number = ['../../robotcar_dw/' '11/2015-11-10-14-15-57'];
run_number = ['../../robotcar_dw/' '08/2015-08-28-09-50-22'];
%run_number = ['../../robotcar_dw/' '02/2015-02-27-18-09-02'];


timestamps = [root_dir run_number '-light/timestamps.txt'];
im_ts = dlmread(timestamps);
n_im = length(im_ts);

mkdir([root_dir run_number '-light/mono_rear']);
mkdir([root_dir run_number '-light/mono_left']);
mkdir([root_dir run_number '-light/mono_right']);
mkdir([root_dir run_number '-light/stereo/centre']);

cameras_dir = {'mono_left/', 'mono_rear/', 'mono_right/', 'stereo/centre/'};
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