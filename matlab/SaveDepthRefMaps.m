function SaveDepthRefMaps( timestamps, cameras, root_dir, correct_rotation, double_scan)
%SAVEDEPTHMAPS Summary of this function goes here
%   Detailed explanation goes here
%   Use Bevilacqua algorithm
    parpool(6);
    
    inpainting_path = '~/Dev/Code/marcoslug-lidar_image/code_matlab';
    addpath(inpainting_path)
    addpath([inpainting_path, '/toolbox_general'])
    addpath([inpainting_path, '/TVL2_classique'])
    addpath([inpainting_path, '/toolbox_signal'])
    
    if ~exist('correct_rotation', 'var')
        correct_rotation = false;
        disp('No rotation correction')
    end
    if ~exist('double_scan', 'var')
        double_scan = false;
        disp('No double scan used')
    else
        double_scan = [root_dir double_scan];
    end
    
    im_ts = dlmread(timestamps);
    n_im = length(im_ts);
    
    mkdir([root_dir 'DepthMap/images/']);
    mkdir([root_dir 'RefMap/images/']);
    
    Params.SaveFlag = 2;
    Params.SavePC = 0;
    Params.eta_d = 1; %1 - 1
    Params.eta_r = 1; %30 - 1 
    Params.Kalpha = 0.03; %0.03 - 0.03
    Params.beta  = .05; %0.6 - 0.3
    Params.lambda_d = 1; %0.5 - 0.1 % 0.34 ok
    Params.lambda_r = 1; %1 - 0.5
    Params.NumIter = 500;
    
    parfor i=1:n_im
        ind_cam = 1;
        if cameras(1) % left
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'mono_left/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);
            
            [u_in, uf, rf, bf] = lidar_mi3s(im, rm, fm, outMask, Params);
            
            if correct_rotation
                rectification_angle_left = 11; % 11 deg
                depthMap = imrotate(uf, rectification_angle_left, 'bilinear', 'loose');
                refMap = imrotate(rf, rectification_angle_left, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(depthMap);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                depthMap = imcrop(depthMap, rect);
                refMap = imcrop(refMap, rect);
            end
            
            imwrite(depthMap, [root_dir 'DepthMap/images/' 'depth_' num2str(i,'%0.6d') '_mono_left.jpg'], 'jpg');
            imwrite(refMap, [root_dir 'RefMap/images/' 'ref_' num2str(i,'%0.6d') '_mono_left.jpg'], 'jpg');
            ind_cam = ind_cam + 1;
        end
        if cameras(2) % rear
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'mono_rear/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);

            [u_in, uf, rf, bf] = lidar_mi3s(im, rm, fm, outMask, Params);

            imwrite(uf, [root_dir 'DepthMap/images/' 'depth_' num2str(i,'%0.6d') '_mono_rear.jpg'], 'jpg');
            imwrite(rf, [root_dir 'RefMap/images/' 'ref_' num2str(i,'%0.6d') '_mono_rear.jpg'], 'jpg');
            ind_cam = ind_cam + 1;
        end
        if cameras(3) % right
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'mono_right/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);
            
            [u_in, uf, rf, bf] = lidar_mi3s(im, rm, fm, outMask, Params);
            
            if correct_rotation
                rectification_angle_right = -11.4; % 11.4 deg
                depthMap = imrotate(uf, rectification_angle_right, 'bilinear', 'loose');
                refMap = imrotate(rf, rectification_angle_right, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(depthMap);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                depthMap = imcrop(depthMap, rect);
                refMap = imcrop(refMap, rect);
            end
            imwrite(depthMap, [root_dir 'DepthMap/images/' 'depth_' num2str(i,'%0.6d') '_mono_right.jpg'], 'jpg');
            imwrite(refMap, [root_dir 'RefMap/images/' 'ref_' num2str(i,'%0.6d') '_mono_right.jpg'], 'jpg');
            ind_cam = ind_cam + 1;
        end
        if cameras(4) % centre
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'stereo/centre/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);

            [u_in, uf, rf, bf] = lidar_mi3s(im, rm, fm, outMask, Params);

            imwrite(uf, [root_dir 'DepthMap/images/' 'depth_' num2str(i,'%0.6d') '_stereo_centre.jpg'], 'jpg');
            imwrite(rf, [root_dir 'RefMap/images/' 'ref_' num2str(i,'%0.6d') '_stereo_centre.jpg'], 'jpg');
        end
        if ~mod(i,5)
            disp(['Saving progression: ' int2str(i*sum(cameras==true)) ' / ' int2str(n_im*sum(cameras==true))])
        end
    end
end

