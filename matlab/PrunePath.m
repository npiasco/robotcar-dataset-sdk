function [ path, timestamps_path ] = PrunePath( X, Y, timestamps, step, check_for_duplicate)
%PRUNEPATH Summary of this function goes here
%   Detailed explanation goes here

	path = {};
    timestamps_path = [];
    
    l = length(X);
    
    d = 0;
    for i = 2:l % Skeeping the first image; most of the time surexposed or blurred
        d = d + norm([X(i); Y(i)] - [X(i-1); Y(i-1)]);
        if d > step
            if check_for_duplicate
                duplicate = false;
                j=length(path);
                while j>0 && ~duplicate
                    if norm([X(i); Y(i)] - path{j}) < step
                        duplicate = true;
                    end
                    j = j-1;
                end
                if ~duplicate
                    path{end+1} = [X(i); Y(i)];
                    timestamps_path(end+1) = timestamps(i);
                    d = 0;
                end
            else
                path{end+1} = [X(i); Y(i)];
                timestamps_path(end+1) = timestamps(i);
                d = 0;    
            end
        end
    end
    
end

