function [im, rm, fm, outMask] = PrepareDataToDepthMapCreation(image_dir, laser_dir, ins_file, models_dir, extrinsics_dir, image_timestamp, opt_laser, disp)
  
% Inspired form ProjectLaserIntoCamera 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  if ~exist('disp', 'var')
    disp = false;
  end
  
  if ~exist('opt_laser', 'var')
    opt_laser = false;
  end

  if image_dir(end) ~= '/'
    image_dir = [image_dir '/'];
  end
  
  if laser_dir(end) ~= '/'
    laser_dir = [laser_dir '/'];
  end
  
  if models_dir(end) ~= '/'
    models_dir = [models_dir '/'];
  end
  
  if extrinsics_dir(end) ~= '/'
    extrinsics_dir = [extrinsics_dir '/'];
  end
  
  laser = regexp(laser_dir, 'lms_front|lms_rear|ldmrs', 'match');
  laser_timestamp = [laser_dir '../' laser{end} '.timestamps'];
  if ~exist(laser_timestamp, 'file')
    laser_timestamp = [laser_dir '../' laser{end} '.timestamps'];
    if ~exist(timestamp_file, 'file')
      error('Timestamp file for laser not found');
    end
  end
  laser_timestamp = dlmread(laser_timestamp);
  
  if opt_laser
      opt_laser_dir = regexp(opt_laser, 'lms_front|lms_rear|ldmrs', 'match');
      opt_laser_timestamp = [opt_laser '../' opt_laser_dir{end} '.timestamps'];
      if ~exist(opt_laser_timestamp, 'file')
        opt_laser_timestamp = [opt_laser '../' opt_laser_dir{end} '.timestamps'];
        if ~exist(opt_laser_timestamp, 'file')
          error('Timestamp file for laser not found');
        end
      end
      opt_laser_timestamp = dlmread(opt_laser_timestamp);
  end

  camera = ...
      regexp(image_dir, '(stereo|mono_left|mono_right|mono_rear)', 'match');
  camera = camera{end};
  
  %TODO: ICP sur les deux pc
  switch camera
      case 'stereo'
          [pointcloud, reflectance] = BuildPointcloud(laser_dir, ins_file, ...
    extrinsics_dir, laser_timestamp(1, 1), laser_timestamp(end, 1), image_timestamp);
          if opt_laser
            [pointcloud2, reflectance2] = BuildPointcloud(opt_laser, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, opt_laser_timestamp(end, 1), image_timestamp);
            pointcloud = [pointcloud pointcloud2];
            reflectance = [reflectance reflectance2];
          end
      case 'mono_left'
          [pointcloud, reflectance] = BuildPointcloud(laser_dir, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, image_timestamp+1e7, image_timestamp);
          if opt_laser
            [pointcloud2, reflectance2] = BuildPointcloud(opt_laser, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, image_timestamp+1e7, image_timestamp);
            pointcloud = [pointcloud pointcloud2];
            reflectance = [reflectance reflectance2];
          end
      case 'mono_right'
          [pointcloud, reflectance] = BuildPointcloud(laser_dir, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, image_timestamp+1e7, image_timestamp);
          if opt_laser
            [pointcloud2, reflectance2] = BuildPointcloud(opt_laser, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, image_timestamp+1e7, image_timestamp);
            pointcloud = [pointcloud pointcloud2];
            reflectance = [reflectance reflectance2];
          end
      case 'mono_rear'
          [pointcloud, reflectance] = BuildPointcloud(laser_dir, ins_file, ...
    extrinsics_dir, laser_timestamp(1,1), laser_timestamp(end,1), image_timestamp);
          if opt_laser
            [pointcloud2, reflectance2] = BuildPointcloud(opt_laser, ins_file, ...
    extrinsics_dir, opt_laser_timestamp(1,1), image_timestamp+1e7, image_timestamp);
            pointcloud = [pointcloud pointcloud2];
            reflectance = [reflectance reflectance2];
          end
  end
  
  extrinsics_path = [extrinsics_dir camera '.txt'];
  if ~exist(extrinsics_path, 'file')
    error(['Camera extrinsics not found at ' extrinsics_path]);
  end
  camera_extrinsics = dlmread(extrinsics_path);
  
  extrinsics_path = [extrinsics_dir 'ins.txt'];
  if ~exist(extrinsics_path, 'file')
    error(['Camera extrinsics not found a ' extrinsics_path]);
  end
  ins_extrinsics = dlmread(extrinsics_path);
  
  if (strfind(ins_file, 'ins.csv'))
    G_camera_ins = SE3MatrixFromComponents(camera_extrinsics) * ...
      SE3MatrixFromComponents(ins_extrinsics);
  else
     % Nathan's hack
    G_camera_ins = SE3MatrixFromComponents(camera_extrinsics);
  end
  
  [fx, fy, cx, cy, G_camera_image, LUT] = ...
      ReadCameraModel(image_dir, models_dir);
  
  image = LoadImage(image_dir, image_timestamp, LUT);
  if ~image
    error(['No image found for timestamp: ' num2str(image_timestamp)]);
  end
  
  % Transform pointcloud into camera image frame
  xyz = (G_camera_image \ G_camera_ins ...
      * [pointcloud; ones(1, size(pointcloud,2))]).';
  xyz(:,4) = [];
  
  % Find which points lie in front of the camera
  in_front = find(xyz(:,3) >= 0);
  
  % Project points into image using a pinhole camera model
  uv = [ fx .* xyz(in_front,1) ./ xyz(in_front,3) + cx, ...
         fy .* xyz(in_front,2) ./ xyz(in_front,3) + cy];
    
  % Find which points have projected within the image bounds
  in_img = (uv(:,1) >= 0.5 & uv(:,1) < size(image,2)-0.5) & ...
    (uv(:,2) >= 0.5 & uv(:,2) < size(image,1)-0.5);
  
  if sum(in_img) == 0
    warning('No points project into image. Is the vehicle stationary?');
  elseif sum(in_img) < 1000
    warning('Very few points project into image. Is the vehicle stationary?');
  end
  
  uv = ceil(uv(in_img,:));
    % Colour pointcloud by depth. Alternately, could use reflectance
  colours = xyz(in_front(in_img), 3);
  reflectance_colours = reflectance(in_front(in_img));
  [h, w, c] = size(image);
  rm = zeros(h, w, 1);
  fm = zeros(h, w, 1);
  outMask = zeros(h, w, 1);
  
  % Filtering huge range data
  m_range = 100;
  colours(colours>m_range)=0;
  
  for i=1:length(uv)
      rm(uv(i,2),uv(i,1)) = colours(i);
      fm(uv(i,2),uv(i,1)) = reflectance_colours(i);
      outMask(uv(i,2),uv(i,1)) = 1; 
  end
  se = strel('disk',25);
  outMask = imdilate(outMask,se);
  
  outMask(floor(h*0.4):end,:) = 1;
  outMask = medfilt2(outMask, [10 10]);
  
  cc = bwconncomp(outMask, 4);
  if cc.NumObjects > 1
    bigger = 1;
    for i=2:cc.NumObjects
        if length(cc.PixelIdxList{i}) > length(cc.PixelIdxList{bigger})
            bigger = i;
        end
    end
    for i=1:cc.NumObjects
        if i ~= bigger
            outMask(cc.PixelIdxList{i}) = 0;
        end
    end
  end
  outMask = logical(outMask == 0);
    cc = bwconncomp(outMask, 4);
  if cc.NumObjects > 1
    bigger = 1;
    for i=2:cc.NumObjects
        if length(cc.PixelIdxList{i}) > length(cc.PixelIdxList{bigger})
            bigger = i;
        end
    end
    for i=1:cc.NumObjects
        if length(cc.PixelIdxList{i}) < length(cc.PixelIdxList{bigger})/2
            outMask(cc.PixelIdxList{i}) = 0;
        end
    end
  end
  
  im = rgb2gray(image);
end
