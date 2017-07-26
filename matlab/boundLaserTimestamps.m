function [ bounded_timestamps ] = boundLaserTimestamps( laser_timestamps, vo_file )
% Bound the laser timestamps by the max vo timestamps
% Usage:  [ bounded_timestamps ] = boundLaserTimestamps( laser_timestamps, ins_file )
%
% INPUTS:
%   laser_timestamps: array of UNIX timestamps from laser
%   vo_file: csv file containing relative pose data
%
% OUTPUTS:
%   bounded_timestamps: is the bounded by max vo_timestamps
%
% Author: Junsheng Fu (junsheng.fu@tut.fi)


vo_file_id = fopen(vo_file);
headers = textscan(vo_file_id, '%s', 8, 'Delimiter',',');
vo_data = textscan(vo_file_id, '%u64 %u64 %f %f %f %f %f %f','Delimiter',',');
fclose(vo_file_id);

vo_timestamps = vo_data{1};

% only need to handle the up bound
end_timestamp = min(vo_timestamps(end), laser_timestamps(end,1));   
end_index = find(laser_timestamps(:,1) <= end_timestamp, 1, 'last');

bounded_timestamps = laser_timestamps(1:end_index, :);

end