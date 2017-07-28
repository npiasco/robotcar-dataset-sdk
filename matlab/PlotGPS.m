function [ X, Y ] =  PlotGPS( gps_file, varargin )
%PLOTGPS Summary of this function goes here
%   Detailed explanation goes here
    
    ins_file_id = fopen(gps_file);
    headers = textscan(ins_file_id, '%s', 12, 'Delimiter',',');
    GPS = textscan(ins_file_id, ...
      '%u64 %u64 %f %f %f %f %f %f %f %f %f %s','Delimiter',',');
    fclose(ins_file_id);

	l = length(GPS{9});
	pivot = [GPS{9}(1) GPS{10}(1)];
	
	X = zeros(l,1);
	Y = zeros(l,1);
	for i = 2:l
		X(i) = pivot(1) - GPS{9}(i);
		Y(i) = pivot(2) - GPS{10}(i);
    end
	
    if ~isempty(varargin)
        figure;
        plot(X,Y);
    end
end

