function CreatDataset( ins_file, path_ref, images_dir, models_dir, step, selected_views, correct_rotation, directory, display)
%CREATDATASET Summary of this function goes here
%   Detailed explanation goes here

    clc;
    if ~exist('selected_views', 'var')
        selected_views = [true true true true]; % left - rear - right - centre
        disp('Selecting all views')
    end
    if ~exist('correct_rotation', 'var')
        correct_rotation = false;
        disp('No rotation correction')
    end
    
    if ~exist('display', 'var')
        display = true;
        disp('Displaying pruned path')
    end
    
    if ~exist('directory', 'var')
        directory = 'Dataset';
        disp('Saving directory is Dataset')
    end


    %% Path creation from ins
    disp('Path creation from ins');

    ins_file_id = fopen(ins_file);
    if strcmp(ins_file(end-6:end-4), 'ins')    
        headers = textscan(ins_file_id, '%s', 15, 'Delimiter',',');
        INS = textscan(ins_file_id, ...
          '%f %s %f %f %f %f %f %f %s %f %f %f %f %f %f','Delimiter',',');        

        origine = INS{1}(1);
        timestamps = INS{1}(1:end-1);
        Poses = InterpolatePoses(ins_file, timestamps', origine);        
    else % VO
        headers = textscan(ins_file_id, '%s', 8, 'Delimiter',',');
        VO = textscan(ins_file_id, ...
            '%f %f %f %f %f %f %f %f','Delimiter',',');

        origine = VO{1}(1);
        timestamps = VO{1}(1:end-1);
        Poses = RelativeToAbsolutePoses(ins_file, timestamps', origine);
    end
    fclose(ins_file_id);

    l = length(Poses);
    X = zeros(l,1);
    Y = zeros(l,1);

    for i = 1:l
        X(i) = Poses{i}(1,4);
        Y(i) = Poses{i}(2,4);
    end
    
    %% Registration across reference path
    disp('Registration across reference path');
    T_to_align = [X Y];
    [Xref, Yref] = PlotGPS(path_ref);
    T_ref = [Xref Yref];
    is_aligned = false;
    while ~is_aligned
        T_aligned = AlignTrajectories( T_to_align, T_ref );
        cont = input('Good registration? (y/n) ', 's');
        if ~strcmp(cont, 'y')
            continue
        else
            is_aligned = true;
        end
    end
    
    % Saving data
    X = T_aligned(:,1);
    Y = T_aligned(:,2);
    
    %% Path pruning with provided step
    disp('Path pruning');
    
    path = {};
    timestamps_path = [];
    
    path{end+1} = [ X(1);  Y(1) ];
    timestamps_path(end+1) = timestamps(1);
    d = 0;
    for i = 2:l
        d = d + norm([X(i); Y(i)] - [X(i-1); Y(i-1)]);
        if d > step
            path{end+1} = [X(i); Y(i)];
            timestamps_path(end+1) = timestamps(i);
            d = 0;
        end
    end
    
    fig = figure;
    if display
        path_l = length(path);
        
        Xp = zeros(path_l,1);
        Yp = zeros(path_l,1);

        for i = 1:path_l
            Xp(i) = path{i}(1);
            Yp(i) = path{i}(2);
        end
        
        plot(X,Y,'b');
        hold on;
        plot(Xp,Yp,'*r');
        legend('Original path','Pruned path');
    end    
    
    disp(['Pruned path with ' int2str(length(path)) ' location (out off ' int2str(l) ')'])
    disp(['Trying to extract ' int2str(length(path)*sum(selected_views==true)) ' images'])
    
    continuing = input('Proceed? (y/n) ', 's');
    close(fig);
    if ~strcmp(continuing, 'y')
        return
    end
    
    %% Closest images extraction according to path
    disp('Closest images extraction');
    cameras = {'mono_left', 'mono_rear', 'mono_right', 'stereo'};
    for i=1:length(cameras)
        disp(['Proceedings ' cameras{i} '...']);
        timestamps_file = [images_dir cameras{i} '.timestamps'];
        images_timestamps = dlmread(timestamps_file);
        
        closest_images_timestamps{i} = GetClosestImages(images_timestamps(:,1), timestamps_path);
    end
    
    %% Images saving and localization file creation    
    disp('Saving images and coord files');
    disp(['Saving images in ' images_dir directory '/images/']);
    mkdir([images_dir directory]);
    mkdir([images_dir directory '/images']);
    
    disp(['Saving coord in ' images_dir directory '/coord.txt']);
    f = fopen([images_dir directory '/coord.txt'], 'wt'); 
    
    disp(['Saving also images_timestamps in ' images_dir directory '/timestamps.txt']);
    f_timestamps = fopen([images_dir directory '/timestamps.txt'], 'wt'); 
    
    disp('Loading camera models...')
    cameras_dir = {'mono_left/', 'mono_rear/', 'mono_right/', 'stereo/centre/'};
    models = cell(size(cameras));
    for i=1:length(cameras)
        [~, ~, ~, ~, ~, models{i}] = ...
            ReadCameraModel([images_dir cameras_dir{i}], models_dir);
    end
    for i=1:length(path)
        
        if selected_views(1)
            image = LoadImage([images_dir cameras_dir{1}],closest_images_timestamps{1}(i),models{1});
            if correct_rotation
                rectification_angle_left = 11; % 11 deg
                image = imrotate(image, rectification_angle_left, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(image);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                image = imcrop(image, rect);
            end
            imwrite(image, [images_dir directory '/images/image_' int2str(i) '_mono_left.png'], 'png');
            fprintf(f_timestamps, '%f\t', closest_images_timestamps{1}(i));
        end
        if selected_views(2)
            image = LoadImage([images_dir cameras_dir{2}],closest_images_timestamps{2}(i),models{2});
            imwrite(image, [images_dir directory '/images/image_' int2str(i) '_mono_rear.png'], 'png');
            fprintf(f_timestamps, '%f\t', closest_images_timestamps{2}(i));

        end
        if selected_views(3)
            image = LoadImage([images_dir cameras_dir{3}],closest_images_timestamps{3}(i),models{3});
            if correct_rotation
                rectification_angle_right = -11.4; % 11.4 deg
                image = imrotate(image, rectification_angle_right, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(image);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                image = imcrop(image, rect);
            end
            imwrite(image, [images_dir directory '/images/image_' int2str(i) '_mono_right.png'], 'png');
            fprintf(f_timestamps, '%f\t', closest_images_timestamps{3}(i));
        end
        if selected_views(4)
            image = LoadImage([images_dir cameras_dir{4}],closest_images_timestamps{4}(i),models{4});
            imwrite(image, [images_dir directory '/images/image_' int2str(i) '_stereo_centre.png'], 'png');
            fprintf(f_timestamps, '%f\t', closest_images_timestamps{4}(i));
        end
        fprintf(f, '%20.18f\t', path{i});
        fprintf(f, '\n');
        fprintf(f_timestamps, '\n');

        if ~mod(i,5)
            disp(['Saving progression: ' int2str(i*sum(selected_views==true)) ' / ' int2str(length(path)*sum(selected_views==true))])
        end
    end
    fclose(f);
    fclose(f_timestamps);
    disp('Done.')
end