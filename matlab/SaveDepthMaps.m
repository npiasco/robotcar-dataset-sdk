function SaveDepthMaps( timestamps, cameras, root_dir, correct_rotation, double_scan)
%SAVEDEPTHMAPS Summary of this function goes here
%   Detailed explanation goes here
    if ~exist('correct_rotation', 'var')
        correct_rotation = false;
        disp('No rotation correction')
    end
    if ~exist('double_scan', 'var')
        double_scan = false;
        disp('No double scan used')
    else
        double_scan = [root_dir 'lms_rear/'];
    end
    
    im = dlmread(timestamps);
    n_im = length(im);
    
    mkdir([root_dir 'DepthMap/']);
    mkdir([root_dir 'DepthMap/gray']);
    mkdir([root_dir 'DepthMap/jet']);
    
    for i=1:n_im
        ind_cam = 1;
        if cameras(1) % left 
            [grayDepthMap, jetDepthMap] = CreatDepthMap([root_dir 'mono_left/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', im(i,ind_cam) , double_scan, false);
            if correct_rotation
                rectification_angle_left = 11; % 11 deg
                grayDepthMap = imrotate(grayDepthMap, rectification_angle_left, 'bilinear', 'loose');
                jetDepthMap = imrotate(jetDepthMap, rectification_angle_left, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(grayDepthMap);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                grayDepthMap = imcrop(grayDepthMap, rect);
                jetDepthMap = imcrop(jetDepthMap, rect);
            end
            imwrite(grayDepthMap, [root_dir 'DepthMap/gray/' 'depth_' num2str(i,'%0.6d') '_mono_left.jpg'], 'jpg');
            imwrite(grayDepthMap, [root_dir 'DepthMap/gray/' 'depth_' num2str(i,'%0.6d') '_mono_left.png'], 'png');
            imwrite(jetDepthMap, [root_dir 'DepthMap/jet/' 'depth_' num2str(i,'%0.6d') '_mono_left.jpg'], 'jpg');
            imwrite(jetDepthMap, [root_dir 'DepthMap/jet/' 'depth_' num2str(i,'%0.6d') '_mono_left.png'], 'png');
            ind_cam = ind_cam + 1;
        end
        if cameras(2) % rear
            [grayDepthMap, jetDepthMap] = CreatDepthMap([root_dir 'mono_rear/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', im(i,ind_cam) , double_scan, false);
            imwrite(grayDepthMap, [root_dir 'DepthMap/gray/' 'depth_' num2str(i,'%0.6d') '_mono_rear.jpg'], 'jpg');
            imwrite(grayDepthMap, [root_dir 'DepthMap/gray/' 'depth_' num2str(i,'%0.6d') '_mono_rear.png'], 'png');
            imwrite(jetDepthMap, [root_dir 'DepthMap/jet/' 'depth_' num2str(i,'%0.6d') '_mono_rear.jpg'], 'jpg');
            imwrite(jetDepthMap, [root_dir 'DepthMap/jet/' 'depth_' num2str(i,'%0.6d') '_mono_rear.png'], 'png');
            ind_cam = ind_cam + 1;
        end
        if cameras(3) % right
            [grayDepthMap, jetDepthMap] = CreatDepthMap([root_dir 'mono_right/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', im(i,ind_cam) , double_scan, false);
            if correct_rotation
                rectification_angle_right = -11.4; % 11.4 deg
                grayDepthMap = imrotate(grayDepthMap, rectification_angle_right, 'bilinear', 'loose');
                jetDepthMap = imrotate(jetDepthMap, rectification_angle_right, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(grayDepthMap);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                grayDepthMap = imcrop(grayDepthMap, rect);
                jetDepthMap = imcrop(jetDepthMap, rect);
            end
            imwrite(grayDepthMap, [root_dir 'DepthMap/gray/' 'depth_' num2str(i,'%0.6d') '_mono_right.jpg'], 'jpg');
            imwrite(grayDepthMap, [root_dir 'DepthMap/gray/' 'depth_' num2str(i,'%0.6d') '_mono_right.png'], 'png');
            imwrite(jetDepthMap, [root_dir 'DepthMap/jet/' 'depth_' num2str(i,'%0.6d') '_mono_right.jpg'], 'jpg');
            imwrite(jetDepthMap, [root_dir 'DepthMap/jet/' 'depth_' num2str(i,'%0.6d') '_mono_right.png'], 'png');
            ind_cam = ind_cam + 1;
        end
        if cameras(4) % centre
            [grayDepthMap, jetDepthMap] = CreatDepthMap([root_dir 'stereo/centre/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', im(i,ind_cam) , double_scan, false);
            imwrite(grayDepthMap, [root_dir 'DepthMap/gray/' 'depth_' num2str(i,'%0.6d') '_stereo_centre.jpg'], 'jpg');
            imwrite(grayDepthMap, [root_dir 'DepthMap/gray/' 'depth_' num2str(i,'%0.6d') '_stereo_centre.png'], 'png');
            imwrite(jetDepthMap, [root_dir 'DepthMap/jet/' 'depth_' num2str(i,'%0.6d') '_stereo_centre.jpg'], 'jpg');
            imwrite(jetDepthMap, [root_dir 'DepthMap/jet/' 'depth_' num2str(i,'%0.6d') '_stereo_centre.png'], 'png');
        end
        if ~mod(i,5)
            disp(['Saving progression: ' int2str(i*sum(cameras==true)) ' / ' int2str(n_im*sum(cameras==true))])
        end
    end
end

