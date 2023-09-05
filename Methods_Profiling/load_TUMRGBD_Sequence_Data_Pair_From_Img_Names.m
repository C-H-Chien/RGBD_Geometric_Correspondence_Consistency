function [rgb1, rgb2, depth1, depth2, R12, T12] = load_TUMRGBD_Sequence_Data_Pair_From_Img_Names ...
         (rgb_time_stamp, depth_time_stamp, rgb_filename, depth_filename, K, RGB_Img_Pair_Names, sequenceName)
    
    dataset_Dir = "/home/chchien/datasets/TUM-RGBD/";
    sequence_path = strcat(dataset_Dir, sequenceName);

    %> Find the corresponding indices for the image pair
    rgb_filename_ = string(rgb_filename); 
    bool_Find_Img1_Index = strcmp(rgb_filename_(:,1), RGB_Img_Pair_Names(1,1));
    bool_Find_Img2_Index = strcmp(rgb_filename_(:,1), RGB_Img_Pair_Names(1,2));
    im1_id = find(bool_Find_Img1_Index == 1);
    im2_id = find(bool_Find_Img2_Index == 1);
    
    %> Fetch the corresponding time stamp
    time1 = rgb_time_stamp(im1_id);
    time2 = rgb_time_stamp(im2_id);
    
    rgb1=imread(strcat(sequence_path, rgb_filename(im1_id,:)));
    rgb2=imread(strcat(sequence_path, rgb_filename(im2_id,:)));
    [s1, pos1] = min(abs(depth_time_stamp - time1));
    [s2, pos2] = min(abs(depth_time_stamp - time2));
    depth1 = imread(strcat(sequence_path,depth_filename(pos1,:)));
    depth2 = imread(strcat(sequence_path,depth_filename(pos2,:)));
    
    %> Get Ground Truth
    ft_filePath = strcat(sequence_path, '/groundtruth.txt');
    gt_file = importdata(ft_filePath).data;
    time_stamps = gt_file(:,1);
    T_gts = gt_file(:,2:4);
    R_gts = gt_file(:,5:8);
    [s1, pos1] = min(abs(time_stamps - time1));
    [s2, pos2] = min(abs(time_stamps - time2));
    C1 = T_gts(pos1,:);
    C2 = T_gts(pos2,:);
    R1 = R_gts(pos1,:);
    R2 = R_gts(pos2,:);
    R1 = transpose(quat2rotm(R1([4 1 2 3])));
    R2 = transpose(quat2rotm(R2([4 1 2 3])));
    R12 = R2 * inv(R1);
    T12 = R2 * (C1' - C2');
    T12x = [0 -T12(3) T12(2);
            T12(3) 0  -T12(1);
            -T12(2) T12(1) 0];

    %> Compute the Fundamental Matrix
    E12 = T12x * R12;
    %T12=C2'-R12*C1';
    F12 = transpose(inv(K)) * E12 * inv(K);
end





