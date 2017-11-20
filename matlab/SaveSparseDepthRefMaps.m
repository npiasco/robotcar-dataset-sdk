function SaveSparseDepthRefMaps( timestamps, cameras, root_dir, correct_rotation, double_scan)
%SAVEDEPTHMAPS Summary of this function goes here
%   Detailed explanation goes here
%   Use Bevilacqua algorithm
    %parpool(24);

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
   
    depth_map_path = 'SparseDepthMap/images/';
    mono_depth_map_path = 'SparseDepthMap/mono_images/';
    ref_map_path = 'SparseRefMap/images/';
    mono_ref_map_path = 'SparseRefMap/mono_images/';
	
    mkdir([root_dir depth_map_path]);
    mkdir([root_dir ref_map_path]);
    mkdir([root_dir mono_depth_map_path]);
    mkdir([root_dir mono_ref_map_path]);
    
    ext = 'png';
    
    parfor i=1:n_im
        ind_cam = 1;

        depth_name_file = [root_dir depth_map_path 'depth_' num2str(i,'%0.6d') '_mono_left.' ext];
        ref_name_file = [root_dir ref_map_path 'ref_' num2str(i,'%0.6d') '_mono_left.' ext];
        mdepth_name_file = [root_dir mono_depth_map_path 'depth_' num2str(i,'%0.6d') '_mono_left.' ext];
        mref_name_file = [root_dir mono_ref_map_path 'ref_' num2str(i,'%0.6d') '_mono_left.' ext];

        if cameras(1) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file') % left
            fprintf(1, 'Proceeding left image number %d', i);
            [rm, fm, rm_coul, fm_coul] = CreatSparseDepthMap([root_dir 'mono_left/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam));
            
            if correct_rotation
                rectification_angle_left = 11; % 11 deg
                depthMap = imrotate(rm_coul, rectification_angle_left, 'nearest', 'loose');
                refMap = imrotate(fm_coul, rectification_angle_left, 'nearest', 'loose');
                mdepthMap = imrotate(rm, rectification_angle_left, 'nearest', 'loose');
                mrefMap = imrotate(fm, rectification_angle_left, 'nearest', 'loose');
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
            imwrite(depthMap, depth_name_file, 'png');
            imwrite(refMap, ref_name_file, 'png');
            imwrite(mdepthMap, mdepth_name_file, 'png');
            imwrite(mrefMap, mref_name_file, 'png');
        end
        if cameras(1)
            ind_cam = ind_cam + 1;
        end
	
        depth_name_file = [root_dir depth_map_path 'depth_' num2str(i,'%0.6d') '_mono_rear.' ext];
        ref_name_file = [root_dir ref_map_path 'ref_' num2str(i,'%0.6d') '_mono_rear.' ext];
        mdepth_name_file = [root_dir mono_depth_map_path 'depth_' num2str(i,'%0.6d') '_mono_rear.' ext];
        mref_name_file = [root_dir mono_ref_map_path 'ref_' num2str(i,'%0.6d') '_mono_rear.' ext];

        if cameras(2) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file')% rear
            fprintf(1, 'Proceeding rear image number %d', i);
            [rm, fm, rm_coul, fm_coul] = CreatSparseDepthMap([root_dir 'mono_rear/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam));
            
            imwrite(rm_coul, depth_name_file, 'png');
            imwrite(fm_coul, ref_name_file, 'png');
            imwrite(rm, mdepth_name_file, 'png');
            imwrite(fm, mref_name_file, 'png');

        end
        if cameras(2)
            ind_cam = ind_cam + 1;
        end
        depth_name_file = [root_dir depth_map_path 'depth_' num2str(i,'%0.6d') '_mono_right.' ext];
        ref_name_file = [root_dir ref_map_path 'ref_' num2str(i,'%0.6d') '_mono_right.' ext];
        mdepth_name_file = [root_dir mono_depth_map_path 'depth_' num2str(i,'%0.6d') '_mono_right.' ext];
        mref_name_file = [root_dir mono_ref_map_path 'ref_' num2str(i,'%0.6d') '_mono_right.' ext];
        if cameras(3) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file')% right
            fprintf(1, 'Proceeding right image number %d', i);
            [rm, fm, rm_coul, fm_coul] = CreatSparseDepthMap([root_dir 'mono_right/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam));
            
            if correct_rotation
                rectification_angle_right = -11.4; % 11.4 deg
                depthMap = imrotate(rm_coul, rectification_angle_right, 'nearest', 'loose');
                refMap = imrotate(fm_coul, rectification_angle_right, 'nearest', 'loose');
                mdepthMap = imrotate(rm, rectification_angle_right, 'nearest', 'loose');
                mrefMap = imrotate(fm, rectification_angle_right, 'nearest', 'loose');
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
            imwrite(depthMap, depth_name_file, 'png');
            imwrite(refMap, ref_name_file, 'png');
            imwrite(mdepthMap, mdepth_name_file, 'png');
            imwrite(mrefMap, mref_name_file, 'png');
        end
        if cameras(3)
            ind_cam = ind_cam + 1;
        end

        depth_name_file = [root_dir depth_map_path 'depth_' num2str(i,'%0.6d') '_stereo_centre.' ext];
        ref_name_file = [root_dir ref_map_path 'ref_' num2str(i,'%0.6d') '_stereo_centre.' ext];
        mdepth_name_file = [root_dir mono_depth_map_path 'depth_' num2str(i,'%0.6d') '_stereo_centre.' ext];
        mref_name_file = [root_dir mono_ref_map_path 'ref_' num2str(i,'%0.6d') '_stereo_centre.' ext];
        if cameras(4) && ~exist(depth_name_file, 'file') && ~exist(ref_name_file, 'file')% centre
            fprintf(1, 'Proceeding front image number %d', i);
            [rm, fm, rm_coul, fm_coul] = CreatSparseDepthMap([root_dir 'stereo/centre/'], ...
                [root_dir 'lms_front/'], [root_dir 'vo/vo.csv'], 'models/', 'extrinsics/', ...
                im_ts(i,ind_cam));
            
            imwrite(rm_coul, depth_name_file, 'png');
            imwrite(fm_coul, ref_name_file, 'png');
            imwrite(rm, mdepth_name_file, 'png');
            imwrite(fm, mref_name_file, 'png');
        end
        if ~mod(i,5)
            disp(['Saving progression: ' int2str(i*sum(cameras==true)) ' / ' int2str(n_im*sum(cameras==true))])
        end
    end
end
