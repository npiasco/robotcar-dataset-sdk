function [ closest_images_timestamps ] = GetClosestImages( images_timestamps, ins_timestamps )
%GETIMAGEPOSE Summary of this function goes here
%   Detailed explanation goes here
    
    closest_images_timestamps = zeros(size(ins_timestamps));
    
    for i=1:length(ins_timestamps)
        %Fast closest research
        [N,bin]=histc(ins_timestamps(i),images_timestamps);
        index=bin+1;
        if bin == 0 || bin == length(images_timestamps)
            if ins_timestamps(i) < images_timestamps(1)
                index = 1;
            else
                index = length(images_timestamps);
            end
        elseif abs(ins_timestamps(i)-images_timestamps(bin))<abs(ins_timestamps(i)-images_timestamps(bin+1))
            index=bin;
        end
        
        closest_images_timestamps(i) = images_timestamps(index);
    end
end

