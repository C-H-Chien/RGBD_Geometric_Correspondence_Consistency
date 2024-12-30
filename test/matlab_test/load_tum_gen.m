function [rgb1,rgb2,depth1,depth2,R12,T12] = load_tum_gen(dataset_root_path, im1_id, im2_id, seq_path)

    associate_file = strcat(dataset_root_path, seq_path, "associate.txt");
    rgb_depth_alignment_data = importdata(associate_file);

    N = length(rgb_depth_alignment_data);
    rgb_time_stamps   = zeros(N, 1);
    depth_time_stamps = zeros(N, 1);
    rgb_filenames     = strings(N, 1);
    depth_filenames   = strings(N, 1);
    for i = 1:length(rgb_depth_alignment_data)
        single_string = rgb_depth_alignment_data{i};
        splitted = split(single_string, ' ');
        rgb_time_stamps(i,1)    = str2num(splitted{1});
        rgb_filenames(i,1)      = splitted{2};
        depth_time_stamps(i,1)  = str2num(splitted{3});
        depth_filenames(i,1)    = splitted{4};
    end




    time1=rgb_time_stamp(im1_id);
    time2=rgb_time_stamp(im2_id);

    rgb1=imread(strcat(dataset_root_path, seq_path, rgb_filename(im1_id,:)));
    rgb2=imread(strcat(dataset_root_path, seq_path, rgb_filename(im2_id,:)));
    [s1,pos1] = min(abs(depth_time_stamp - time1));
    [s2,pos2] = min(abs(depth_time_stamp - time2));
    depth1=imread(strcat(dataset_root_path, seq_path, depth_filename(pos1,:)));
    depth2=imread(strcat(dataset_root_path, seq_path, depth_filename(pos2,:)));
    %> Get Ground Truth
    ft_filePath=strcat(dataset_root_path, seq_path, "groundtruth.txt");
    gt_file = importdata(ft_filePath).data;
%     gt_file(1:3, :) = [];
    time_stamps = gt_file(:,1);
    T_gts = gt_file(:,2:4);
    R_gts = gt_file(:,5:8);
    [s1,pos1] = min(abs(time_stamps - time1));
    [s2,pos2] = min(abs(time_stamps - time2));
    C1 = T_gts(pos1,:);
    C2 = T_gts(pos2,:);
    R1 = R_gts(pos1,:);
    R2 = R_gts(pos2,:);
    R1 = transpose(quat2rotm(R1([4 1 2 3])));
    R2 = transpose(quat2rotm(R2([4 1 2 3])));
    R12 = R2 * inv(R1);
    T12 = R2 * (C1' - C2');

end