function CreatDataset( ins_file, images_dir, models_dir, step )
%CREATDATASET Summary of this function goes here
%   Detailed explanation goes here

    display = true;

    % Path creation from ins
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
    
    % Path pruning with provided step
    disp('Path pruning');
    
    path = {};
    timestamps_path = [];
    
    path{end+1} = [Poses{1}(1,4); Poses{1}(2,4)];
    timestamps_path(end+1) = timestamps(1);
    d = 0;
    for i = 2:l
        d = d + norm([Poses{i}(1,4); Poses{i}(2,4)] - [Poses{i-1}(1,4); Poses{i-1}(2,4)]);
        if d > step
            path{end+1} = [Poses{i}(1,4); Poses{i}(2,4)];
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
    disp(['Trying to extract ' int2str(length(path)*4) ' images'])
    
    continuing = input('Proceed? (y/n) ', 's');
    close(fig);
    if ~strcmp(continuing, 'y')
        return
    end
    
    % Closest images extraction according to path
    disp('Closest images extraction');
    cameras = {'mono_left', 'mono_rear', 'mono_right', 'stereo'};
    for i=1:length(cameras)
        timestamps_file = [images_dir cameras{i} '.timestamps'];
        images_timestamps = dlmread(timestamps_file);
        
        closest_images_timestamps{i} = GetClosestImages(images_timestamps(:,1), timestamps_path);
    end
    
    % Images saving and localization file creation    
    disp('Saving images and coord files');
    disp(['Saving images in ' images_dir 'Dataset/images/']);
    mkdir([images_dir 'Dataset']);
    mkdir([images_dir 'Dataset/images']);
    
    disp(['Saving coord in ' images_dir 'Dataset/coord.txt']);
    f = fopen([images_dir 'Dataset/coord.txt'], 'wt');
    for i=4403 : 10343
        for jx = 1 : 5
            
        end
    end

    

    
    disp('Loading camera models...')
    cameras_dir = {'mono_left/', 'mono_rear/', 'mono_right/', 'stereo/centre/'};
    models = {};
    for i=1:length(cameras)
        [~, ~, ~, ~, ~, models{i}] = ...
            ReadCameraModel([images_dir cameras_dir{i}], models_dir);
    end
    for i=1:length(path)
        image = LoadImage([images_dir cameras_dir{1}],closest_images_timestamps{1}(i),models{1});
        imwrite(image, [images_dir 'Dataset/images/image_' int2str(i) '_mono_left.png'], 'png');
        image = LoadImage([images_dir cameras_dir{2}],closest_images_timestamps{2}(i),models{2});
        imwrite(image, [images_dir 'Dataset/images/image_' int2str(i) '_mono_rear.png'], 'png');
        image = LoadImage([images_dir cameras_dir{3}],closest_images_timestamps{3}(i),models{3});
        imwrite(image, [images_dir 'Dataset/images/image_' int2str(i) '_mono_right.png'], 'png');
        image = LoadImage([images_dir cameras_dir{4}],closest_images_timestamps{4}(i),models{4});
        imwrite(image, [images_dir 'Dataset/images/image_' int2str(i) '_stereo_centre.png'], 'png');
        
        fprintf(f, '%20.18f\t', path{i});
        fprintf(f, '\n');
        
        if ~mod(i,5)
            disp(['Saving progression: ' int2str(i*4) ' / ' int2str(length(path)*4)])
        end
    end
    fclose(f);
    disp('Done.')
end

