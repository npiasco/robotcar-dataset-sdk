close all;

root = '../training/';
files = {'TrainDataset_02_10_15/', 'TrainDataset_05_19_15/', 'TrainDataset_08_28_15/', 'TrainDataset_11_10_15/'};
north = [0,1];
for f=1:length(files)
    traj = load([root, files{f}, 'coord.txt']);
    [n, dim] = size(traj);
    bearings = zeros(n,1);
    for i = 2:n-1
        v = [traj(i+1,1) - traj(i-1,1), traj(i+1,2) - traj(i-1,2)];
        v = v/norm(v);
        bearings(i) = acos( dot(v,north) ); % between 0 - pi
    end
    bearings(1) = bearings(2);
    bearings(end) = bearings(end-1);
    
    csvwrite([root, files{f}, 'coordbearing.txt'], [traj(:,1), traj(:,2), bearings]);
    
    coordxIm = zeros(n*4,3);
    for i = 1:4:length(coordxIm)
        id = round(i/4+1);
        for j=i:i+3
            coordxIm(j,:) = [traj(id, 1), traj(id, 2), bearings(id)];
        end
        coordxIm(i,3) = mod(bearings(id) - 7*pi/32, pi); % left
        coordxIm(i+1,3) = mod(bearings(id) + pi/2, pi); % rear
        coordxIm(i+2,3) = mod(bearings(id) + 7*pi/32, pi); % right        
    end
    
    csvwrite([root, files{f}, 'coordxImbearing.txt'], coordxIm);
end