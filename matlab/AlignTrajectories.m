function T3 = AlignTrajectories( T_to_align, T_ref )
%ALIGNTRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here
    fig = figure;
    plot(T_ref(:,1), T_ref(:,2), 'r');
    hold on;
    plot(T_to_align(:,1), T_to_align(:,2), 'b');
    legend('Target','To align');

    e_angle = input('Approximate rotation: ', 's');
    e_angle = str2double(e_angle);
    T_ref_begin = input('T_ref begin: ', 's');
    T_ref_begin = str2num(T_ref_begin);
    T_ref_end = input('T_ref end: ', 's');
    T_ref_end = str2num(T_ref_end);
    
    T_ref = T_ref(T_ref_begin:end-T_ref_end,:);

    l1 = length(T_to_align);
    l2 = length(T_ref);
%     V1 = T_to_align(end,:) - T_to_align(1,:);
%     V1 = V1/norm(V1);
%     V2 = T_ref(end,:) - T_ref(1,:);
%     V2 = V2/norm(V2);
%     
%     e_angle = asin( V1(1)*V2(2) - V2(1)*V1(2) );

    Mrot = [cos(e_angle) -sin(e_angle) 0 0; ...
            sin(e_angle) cos(e_angle) 0 0; ...
            0 0 1 0; ...
            0 0 0 1]; % Pure Rot along z
    
    pc_to_align = pointCloud([T_to_align(:,1) T_to_align(:,2) zeros(l1,1)]);
    pc_ref =  pointCloud([T_ref(:,1) T_ref(:,2) zeros(l2,1)]);
    
    pruned_pc_to_align = pcdownsample(pc_to_align, 'random', 0.5);
    pruned_pc_ref =  pcdownsample(pc_ref, 'random', 0.5);
    
    [tform, movingReg] = pcregrigid(pruned_pc_to_align, pruned_pc_ref, 'Verbose', false, 'InitialTransform', affine3d(Mrot));
    
    aligned = pctransform(pc_to_align, tform);
    aligned = aligned.Location;
    T3 = [aligned(:,1) aligned(:,2)];
    
    close(fig);
    figure;
    plot(T_ref(:,1), T_ref(:,2), 'r');
    hold on;
    plot(T3(:,1), T3(:,2), 'g');
    plot(T_to_align(:,1), T_to_align(:,2), 'b');
    legend('Target','Aligned','Original');
end

