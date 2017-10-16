function SaveDepthRefMaps( timestamps, cameras, root_dir, correct_rotation, double_scan)
%SAVEDEPTHMAPS Summary of this function goes here
%   Detailed explanation goes here
%   Use Bevilacqua algorithm
    parpool(24);

    inpainting_path = '~/Dev/marcos-lidar_image/';
    addpath(inpainting_path)
    addpath([inpainting_path, '/toolbox_general'])
    addpath([inpainting_path, '/TVL2_classique'])
    
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
    mkdir([root_dir 'DepthMap/mono_images/']);
    mkdir([root_dir 'RefMap/mono_images/']);
    
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

        depth_name_file = [root_dir 'DepthMap/images/' 'depth_' num2str(i,'%0.6d') '_mono_left.jpg'];
        ref_name_file = [root_dir 'RefMap/images/' 'ref_' num2str(i,'%0.6d') '_mono_left.jpg'];
        mdepth_name_file = [root_dir 'DepthMap/mono_images/' 'depth_' num2str(i,'%0.6d') '_mono_left.jpg'];
        mref_name_file = [root_dir 'RefMap/mono_images/' 'ref_' num2str(i,'%0.6d') '_mono_left.jpg'];

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
                depthMap = imrotate(uf, rectification_angle_left, 'bilinear', 'loose');
                refMap = imrotate(rf, rectification_angle_left, 'bilinear', 'loose');
                mdepthMap = imrotate(mono_uf, rectification_angle_left, 'bilinear', 'loose');
                mrefMap = imrotate(mono_rf, rectification_angle_left, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(depthMap);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                depthMap = imcrop(depthMap, rect);
                refMap = imcrop(refMap, rect);
                mdepthMap = imcrop(mdepthMap, rect);
                mrefMap = imcrop(mrefMap, rect);
            end
            imwrite(depthMap, depth_name_file, 'jpg');
            imwrite(refMap, ref_name_file, 'jpg');
            imwrite(mdepthMap, mdepth_name_file, 'jpg');
            imwrite(mrefMap, mref_name_file, 'jpg');
        end
        if cameras(1)
            ind_cam = ind_cam + 1;
        end
	
        depth_name_file = [root_dir 'DepthMap/images/' 'depth_' num2str(i,'%0.6d') '_mono_rear.jpg'];
        ref_name_file = [root_dir 'RefMap/images/' 'ref_' num2str(i,'%0.6d') '_mono_rear.jpg'];
        mdepth_name_file = [root_dir 'DepthMap/mono_images/' 'depth_' num2str(i,'%0.6d') '_mono_rear.jpg'];
        mref_name_file = [root_dir 'RefMap/mono_images/' 'ref_' num2str(i,'%0.6d') '_mono_rear.jpg'];

        if cameras(2) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file')% rear
            fprintf(1, 'Proceeding rear image number %d', i);
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'mono_rear/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);
            if (rm == 0); rm(:,:) = 1; end
            if (fm == 0); fm(:,:) = 1; end
            [u_in, uf, rf, bf, mono_uf, mono_rf] = lidar_mi3s(im, rm, fm, outMask, Params);

            imwrite(uf, depth_name_file, 'jpg');
            imwrite(rf, ref_name_file, 'jpg');
            imwrite(mono_uf, mdepth_name_file, 'jpg');
            imwrite(mono_rf, mref_name_file, 'jpg');

        end
        if cameras(2)
            ind_cam = ind_cam + 1;
        end
        depth_name_file = [root_dir 'DepthMap/images/' 'depth_' num2str(i,'%0.6d') '_mono_right.jpg'];
        ref_name_file = [root_dir 'RefMap/images/' 'ref_' num2str(i,'%0.6d') '_mono_right.jpg'];
        mdepth_name_file = [root_dir 'DepthMap/mono_images/' 'depth_' num2str(i,'%0.6d') '_mono_right.jpg'];
        mref_name_file = [root_dir 'RefMap/mono_images/' 'ref_' num2str(i,'%0.6d') '_mono_right.jpg'];
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
                depthMap = imrotate(uf, rectification_angle_right, 'bilinear', 'loose');
                refMap = imrotate(rf, rectification_angle_right, 'bilinear', 'loose');
                mdepthMap = imrotate(mono_uf, rectification_angle_right, 'bilinear', 'loose');
                mrefMap = imrotate(mono_rf, rectification_angle_right, 'bilinear', 'loose');
                scale = 0.72;
                [W, H, c] = size(depthMap);
                Wcroop = floor(W * scale);
                Hcroop = floor(H * scale);
                rect = [floor((W-Wcroop)/2) floor((H-Hcroop)/2) Wcroop Hcroop];
                depthMap = imcrop(depthMap, rect);
                refMap = imcrop(refMap, rect);
                mdepthMap = imcrop(mdepthMap, rect);
                mrefMap = imcrop(mrefMap, rect);
            end
            imwrite(depthMap, depth_name_file, 'jpg');
            imwrite(refMap, ref_name_file, 'jpg');
            imwrite(mdepthMap, mdepth_name_file, 'jpg');
            imwrite(mrefMap, mref_name_file, 'jpg');
        end
        if cameras(3)
            ind_cam = ind_cam + 1;
        end

        depth_name_file = [root_dir 'DepthMap/images/' 'depth_' num2str(i,'%0.6d') '_stereo_centre.jpg'];
        ref_name_file = [root_dir 'RefMap/images/' 'ref_' num2str(i,'%0.6d') '_stereo_centre.jpg'];
        mdepth_name_file = [root_dir 'DepthMap/mono_images/' 'depth_' num2str(i,'%0.6d') '_stereo_centre.jpg'];
        mref_name_file = [root_dir 'RefMap/mono_images/' 'ref_' num2str(i,'%0.6d') '_stereo_centre.jpg'];
        if cameras(4) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file')% centre
            fprintf(1, 'Proceeding front image number %d', i);
            [im, rm, fm, outMask] = PrepareDataToDepthMapCreation([root_dir 'stereo/centre/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam) , double_scan, false);
            if (rm == 0); rm(:,:) = 1; end
            if (fm == 0); fm(:,:) = 1; end
            [u_in, uf, rf, bf, mono_uf, mono_rf] = lidar_mi3s(im, rm, fm, outMask, Params);

            imwrite(uf, depth_name_file, 'jpg');
            imwrite(rf, ref_name_file, 'jpg');
            imwrite(mono_uf, mdepth_name_file, 'jpg');
            imwrite(mono_rf, mref_name_file, 'jpg');
        end
        if ~mod(i,5)
            disp(['Saving progression: ' int2str(i*sum(cameras==true)) ' / ' int2str(n_im*sum(cameras==true))])
        end
    end
end
