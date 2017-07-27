function PlotGPS( gps_file )
%PLOTGPS Summary of this function goes here
%   Detailed explanation goes here
    
    ins_file_id = fopen(gps_file);
    headers = textscan(ins_file_id, '%s', 12, 'Delimiter',',');
    GPS = textscan(ins_file_id, ...
      '%u64 %u64 %f %f %f %f %f %f %f %f %f %s','Delimiter',',');
    fclose(ins_file_id);

    figure;
    plot(GPS{3}(:),GPS{4}(:));

end

