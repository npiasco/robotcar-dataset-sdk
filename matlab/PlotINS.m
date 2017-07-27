function [ X, Y ] = PlotINS( ins_file, varargin )
%PLOTVO Summary of this function goes here
%   Detailed explanation goes here
    

    ins_file_id = fopen(ins_file);
    headers = textscan(ins_file_id, '%s', 15, 'Delimiter',',');
    INS = textscan(ins_file_id, ...
          '%u64 %s %f %f %f %f %f %f %s %f %f %f %f %f %f','Delimiter',',');
    fclose(ins_file_id);
    origine = INS{1}(1);
  % Load transforms between laser/ins and vehicle
    timestamps = INS{1}(1:end-1);

    
    Poses = InterpolatePoses(ins_file, timestamps', origine);
    l = length(Poses);
    X = zeros(l,1);
    Y = zeros(l,1);

    for i = 1:l
        
        X(i) = Poses{i}(1,4);
        Y(i) = Poses{i}(2,4);
    end
    
    if ~isempty(varargin)
        figure;
        plot(X,Y);
    end
end