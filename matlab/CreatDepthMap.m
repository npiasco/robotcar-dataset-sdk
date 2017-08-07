function [grayDepthMap, jetDepthMap] = CreatDepthMap(image_dir, laser_dir, ins_file, models_dir, extrinsics_dir, image_timestamp, opt_laser, disp)
  
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
  
%   if ~exist('image_timestamp', 'var')
%     % Get timestamps from file, to avoid having to list whole image
%     % directory
%     camera = regexp(image_dir, 'stereo|mono_left|mono_right|mono_rear', 'match');
%     timestamp_file = [image_dir '../' camera{end} '.timestamps'];
%     if ~exist(timestamp_file, 'file')
%       timestamp_file = [image_dir '../../' camera{end} '.timestamps'];
%       if ~exist(timestamp_file, 'file')
%         error('Timestamp file for camera not found');
%       end
%     end
%     timestamps = dlmread(timestamp_file);
%      % Search for first chunk with data
%     for chunk = 1:timestamps(end,2)
%       timestamp_index = find(timestamps(:,2) == chunk, 1, 'first');
%       if isempty(timestamp_index)
%         error('No laser scans found in specified directory');
%       end
%       image_timestamp = timestamps(timestamp_index, 1);
%       if exist([image_dir num2str(image_timestamp) '.png'], 'file')
%         break;
%       end
%     end
%   end
  
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
    extrinsics_dir, image_timestamp-1e7, laser_timestamp(end, 1), image_timestamp);
          if opt_laser
            [pointcloud2, reflectance] = BuildPointcloud(opt_laser, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, opt_laser_timestamp(end, 1), image_timestamp);
            pointcloud = [pointcloud pointcloud2];
          end
      case 'mono_left'
          [pointcloud, reflectance] = BuildPointcloud(laser_dir, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, image_timestamp+1e7, image_timestamp);
          if opt_laser
            [pointcloud2, reflectance] = BuildPointcloud(opt_laser, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, image_timestamp+1e7, image_timestamp);
            pointcloud = [pointcloud pointcloud2];
          end
      case 'mono_right'
          [pointcloud, reflectance] = BuildPointcloud(laser_dir, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, image_timestamp+1e7, image_timestamp);
          if opt_laser
            [pointcloud2, reflectance] = BuildPointcloud(opt_laser, ins_file, ...
    extrinsics_dir, image_timestamp-1e7, image_timestamp+1e7, image_timestamp);
            pointcloud = [pointcloud pointcloud2];
          end
      case 'mono_rear'
          [pointcloud, reflectance] = BuildPointcloud(laser_dir, ins_file, ...
    extrinsics_dir, laser_timestamp(1,1), image_timestamp+1e7, image_timestamp);
          if opt_laser
            [pointcloud2, reflectance] = BuildPointcloud(opt_laser, ins_file, ...
    extrinsics_dir, opt_laser_timestamp(1,1), image_timestamp+1e7, image_timestamp);
            pointcloud = [pointcloud pointcloud2];
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
  
  factor = 0.4;
  uv = ceil(uv(in_img,:)*factor);
    % Colour pointcloud by depth. Alternately, could use reflectance
  colours = xyz(in_front(in_img), 3);
  u_colours = colours;
  u_colours(u_colours>150) = 150;
  u_colours = u_colours / max(u_colours);
  [X ind] = sort(colours, 'descend');
  [h, w, c] = size(image);
  dim = zeros(round(h*factor), round(w*factor), 3);
  step = 1;
  
  raw_colours = zeros(length(ind(1:step:end)),3);
  for i=1:length(ind(1:step:end))
      %raw_colours(i,:) = tab_jet(round(64*u_colours(ind(i))),:)*255;
      val = round(u_colours(ind(i))*255);
      raw_colours(i,:) = ones(3,1)*val;
  end
  dim_circle = insertShape(dim, 'FilledCircle', ...
      [uv(ind(1:step:end),:) ones(length(ind(1:step:end)),1)*5], ...
      'Color', raw_colours, 'Opacity', 1);
  
  unitone_dim = dim_circle(:,:,1)/255;
  med = medfilt2(unitone_dim, [11,11]);
  med(med==0) = nan; 

  imwrite(med,'test.png', 'png'); 
  if disp
      clf;
      figure
      subplot(2,2,1)
      imshow(imresize(image, factor));
      subplot(2,2,2)
      imshow(unitone_dim)
      subplot(2,2,3)
      imshow(med)
      subplot(2,2,4)
      imshow(imresize(image, factor));
      colormap jet;
      hold on;
      scatter(uv(:,1),uv(:,2), 90, colours, '.');
  end
  
  grayDepthMap = med;
  jetDepthMap = ind2rgb(gray2ind(med,1024), jet(1024));

end
