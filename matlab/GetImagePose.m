function [ X_im, Y_im ] = GetImagePose( images_timestamps, ins_file )
%GETIMAGEPOSE Summary of this function goes here
%   Detailed explanation goes here
    
    X_im = zeros(size(images_timestamps));
    Y_im = zeros(size(images_timestamps));

    ins_file_id = fopen(ins_file);
    if strcmp(ins_file(end-6:end-4), 'ins')    
        headers = textscan(ins_file_id, '%s', 15, 'Delimiter',',');
        INS = textscan(ins_file_id, ...
          '%f %s %f %f %f %f %f %f %s %f %f %f %f %f %f','Delimiter',',');        

        origine = INS{1}(1);
        timestamps = INS{1}(1:end-1);
        Poses = InterpolatePoses(ins_file, timestamps', origine);        
        
    else % VO
        headers = textscan(ins_file_id, '%s', 8, 'Delimiter',',');
        VO = textscan(ins_file_id, ...
            '%f %f %f %f %f %f %f %f','Delimiter',',');

        origine = VO{1}(1);
        timestamps = VO{1}(1:end-1);
        Poses = RelativeToAbsolutePoses(ins_file, timestamps', origine);
    end
    fclose(ins_file_id);

    l = length(Poses);
    X = zeros(l,1);
    Y = zeros(l,1);

    for i = 1:l
        X(i) = Poses{i}(1,4);
        Y(i) = Poses{i}(2,4);
    end
    
    for i=1:length(images_timestamps)
        %Fast closest research
        [N,bin]=histc(images_timestamps(i),timestamps);
        index=bin+1;
        if bin == 0 || bin == length(timestamps)
            if images_timestamps(i) < timestamps(1)
                index = 1;
            else
                index = length(timestamps);
            end
        else if abs(images_timestamps(i)-timestamps(bin))<abs(images_timestamps(i)-timestamps(bin+1))
            index=bin;
        end
        
        X_im(i) = X(index);
        Y_im(i) = Y(index);
    end
end

