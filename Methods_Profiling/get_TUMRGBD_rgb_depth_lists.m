function [rgb_time_stamp, depth_time_stamp, rgb_filename, depth_filename] = ...
         get_TUMRGBD_rgb_depth_lists(sequenceName)
    
    dataset_Dir = "/home/chchien/datasets/TUM-RGBD/";
    sequence_path = strcat(dataset_Dir, sequenceName);

    imageList = strcat(sequence_path, '/rgb.txt');
    imageList = importdata(imageList);
    depthList = strcat(sequence_path, '/depth.txt');
    depthList = importdata(depthList);

    imageList(1,:) = [];
    imageList(1,:) = [];
    imageList(1,:) = [];

    depthList(1,:) = [];
    depthList(1,:) = [];
    depthList(1,:) = [];

    rgb_time_stamp = [];
    depth_time_stamp = [];
    rgb_filename = [];
    depth_filename = [];
    for i = 1:min(length(imageList),length(depthList))
        single_string = imageList{i};
        splitted = split(single_string, ' ');

        rgb_stamp_single = str2num(splitted{1});
        rgb_filename_single = splitted{2};
        single_string=depthList{i};
        splitted = split(single_string, ' ');
        depth_stamp_single = str2num(splitted{1});
        depth_filename_single = splitted{2};

        rgb_time_stamp = [rgb_time_stamp; rgb_stamp_single];
        depth_time_stamp = [depth_time_stamp; depth_stamp_single];
        rgb_filename = [rgb_filename; rgb_filename_single];
        depth_filename = [depth_filename; depth_filename_single];
    end
end





