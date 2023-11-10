function [rgb1, rgb2, depth1, depth2, R12, T12] = load_ICL_NUIM_Sequence_Data_Pair ...
         (PARAMS, im1_id, im2_id, rgb_filename, depth_filename, K)
    
    sequence_path = strcat(PARAMS.DATASET_PATH, PARAMS.SEQUENCE_NAME);

    %> Read RGB images
    rgb1 = imread(rgb_filename(im1_id,:));
    rgb2 = imread(rgb_filename(im2_id,:));
    
    %> Read Depth Images
    depth1 = imread(depth_filename(im1_id,:));
    depth2 = imread(depth_filename(im2_id,:));
    
    %> Get Ground Truth
    ft_filePath = strcat(sequence_path, 'groundtruth.txt');
    gt_file = importdata(ft_filePath);
    time_stamps = gt_file(:,1);
    T_gts = gt_file(:,2:4);
    R_gts = gt_file(:,5:8);
    pos1 = im1_id;
    pos2 = im2_id;
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





