function CompareVOtoINS( ins_file, vo_file, gps_file )
%COMPAREVOTOINS Summary of this function goes here
%   Detailed explanation goes here
    [X_VO, Y_VO] = PlotVO(vo_file);
    [X_INS, Y_INS] = PlotINS(ins_file);
    
    figure
    plot(X_VO, Y_VO, 'r');
    hold on;
    plot(X_INS, Y_INS, 'b');
    
    if exist('gps_file', 'var')
        [X_GPS, Y_GPS] = PlotGPS(gps_file);
        plot(X_GPS, Y_GPS, 'g');
        legend('VO', 'INS', 'GPS');
    else
        legend('VO', 'INS');
    end
    hold off;
end