function CompareVOtoINS( ins_file, vo_file )
%COMPAREVOTOINS Summary of this function goes here
%   Detailed explanation goes here
    [X_VO, Y_VO] = PlotVO(vo_file);
    [X_INS, Y_INS] = PlotINS(ins_file);
    
    figure
    plot(X_VO, Y_VO, 'r');
    hold on;
    plot(X_INS, Y_INS, 'b');
    legend('VO', 'INS');
    hold off;
end

