function TrueDepthRefMaps( timestamps, cameras, root_dir, correct_rotation, double_scan)
%SAVEDEPTHMAPS Summary of this function goes here
%   Detailed explanation goes here
%   Use Bevilacqua algorithm
% 	Save depth maps png-uin16 in cm
%   Save ref map in percentages * 1000 (according to max ref = 2048)
    %parpool(24);

    inpainting_path = '~/Dev/marcos-lidar_image';
    addpath(inpainting_path)
    addpath([inpainting_path, '/toolbox_general'])
    addpath([inpainting_path, '/TVL2_classique'])
    %addAttachedFiles(pool, [inpainting_path, '/TVL2_classique/gradient_mex.mexw64'])
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
    
    pref = 'TrueValues';
    depth_map_path = [pref 'DepthMap/'];
    ref_map_path = [pref 'RefMap/'];
	
    mkdir([root_dir depth_map_path]);
    mkdir([root_dir ref_map_path]);
    
    Params.SaveFlag = 2;
    Params.SavePC = 0;
    Params.eta_d = 1; %1 - 1
    Params.eta_r = 1; %30 - 1 
    Params.Kalpha = 0.03; %0.03 - 0.03
    Params.beta  = .05; %0.6 - 0.3
    Params.lambda_d = 1; %0.5 - 0.1 % 0.34 ok
    Params.lambda_r = 1; %1 - 0.5
    Params.NumIter = 500;
%    Params.NoMask = false;
    
    parfor i=1:n_im
        ind_cam = 1;

        depth_name_file = [root_dir depth_map_path 'depth_' num2str(i,'%0.6d') '_mono_left.png'];
        ref_name_file = [root_dir ref_map_path 'ref_' num2str(i,'%0.6d') '_mono_left.png'];

        if cameras(1) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file') % left
            fprintf(1, 'Proceeding left image number %d', i);
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'mono_left/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);
    
            if (rm == 0); rm(:,:) = 1; end
            if (fm == 0); fm(:,:) = 1; end

            [u_in, uf, rf, bf, mono_uf, mono_rf] = lidar_mi3s(im, rm, fm, outMask, Params);
            
            if correct_rotation
                rectification_angle_left = 11; % 11 deg
                depthMap = imrotate(u_in, rectification_angle_left, 'bilinear', 'loose');
                refMap = imrotate(mono_rf, rectification_angle_left, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(depthMap);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                depthMap = imcrop(depthMap, rect);
                refMap = imcrop(refMap, rect);

            end
            imwrite(uint16(depthMap*100)), depth_name_file, 'png', 'BitDepth', 16));
            imwrite(uint16(refMap/2048*1000)), ref_name_file, 'png', 'BitDepth', 16));

        end
        if cameras(1)
            ind_cam = ind_cam + 1;
        end
	
        depth_name_file = [root_dir depth_map_path 'depth_' num2str(i,'%0.6d') '_mono_rear.png'];
        ref_name_file = [root_dir ref_map_path 'ref_' num2str(i,'%0.6d') '_mono_rear.png'];

        if cameras(2) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file')% rear
            fprintf(1, 'Proceeding rear image number %d', i);
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'mono_rear/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);
            if (rm == 0); rm(:,:) = 1; end
            if (fm == 0); fm(:,:) = 1; end
            [u_in, uf, rf, bf, mono_uf, mono_rf] = lidar_mi3s(im, rm, fm, outMask, Params);

            imwrite(uint16(u_in*100)), depth_name_file, 'png', 'BitDepth', 16));
            imwrite(uint16(mono_rf/2048*1000)), ref_name_file, 'png', 'BitDepth', 16));


        end
        if cameras(2)
            ind_cam = ind_cam + 1;
        end
        depth_name_file = [root_dir depth_map_path 'depth_' num2str(i,'%0.6d') '_mono_right.png'];
        ref_name_file = [root_dir ref_map_path 'ref_' num2str(i,'%0.6d') '_mono_right.png'];

        if cameras(3) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file')% right
            fprintf(1, 'Proceeding right image number %d', i);
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'mono_right/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);
            if (rm == 0); rm(:,:) = 1; end
            if (fm == 0); fm(:,:) = 1; end
            
            [u_in, uf, rf, bf, mono_uf, mono_rf] = lidar_mi3s(im, rm, fm, outMask, Params);
            
            if correct_rotation
                rectification_angle_right = -11.4; % 11.4 deg
                depthMap = imrotate(u_in, rectification_angle_right, 'bilinear', 'loose');
                refMap = imrotate(mono_rf, rectification_angle_right, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(depthMap);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                depthMap = imcrop(depthMap, rect);
                refMap = imcrop(refMap, rect);

            end
            imwrite(uint16(depthMap*100)), depth_name_file, 'png', 'BitDepth', 16));
            imwrite(uint16(refMap/2048*1000)), ref_name_file, 'png', 'BitDepth', 16));
        end
        if cameras(3)
            ind_cam = ind_cam + 1;
        end

        depth_name_file = [root_dir depth_map_path 'depth_' num2str(i,'%0.6d') '_stereo_centre.png'];
        ref_name_file = [root_dir ref_map_path 'ref_' num2str(i,'%0.6d') '_stereo_centre.png'];

        if cameras(4) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file')% centre
            fprintf(1, 'Proceeding front image number %d', i);
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'stereo/centre/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);
            if (rm == 0); rm(:,:) = 1; end
            if (fm == 0); fm(:,:) = 1; end
            [u_in, uf, rf, bf, mono_uf, mono_rf] = lidar_mi3s(im, rm, fm, outMask, Params);

            imwrite(uint16(u_in*100)), depth_name_file, 'png', 'BitDepth', 16));
            imwrite(uint16(mono_rf/2048*1000)), ref_name_file, 'png', 'BitDepth', 16));
        end
        if ~mod(i,5)
            disp(['Saving progression: ' int2str(i*sum(cameras==true)) ' / ' int2str(n_im*sum(cameras==true))])
        end
    end
end
