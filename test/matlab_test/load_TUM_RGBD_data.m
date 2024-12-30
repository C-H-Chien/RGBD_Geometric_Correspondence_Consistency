function [rgb_filenames, depth_filenames, GT_rotations, GT_translations, N] = load_TUM_RGBD_data(dataset_root_path, seq_path)

    associate_file = strcat(dataset_root_path, seq_path, "associate.txt");
    GT_file = strcat(dataset_root_path, seq_path, "groundtruth.txt");
    rgb_depth_alignment_data = importdata(associate_file);
    GT_poses = importdata(GT_file).data;
    GT_time_stamps = GT_poses(:,1);
    GT_quaternions_data = GT_poses(:,2:5);
    GT_translations_data = GT_poses(:,6:8);

    N = length(rgb_depth_alignment_data);
    rgb_time_stamps   = zeros(N, 1);
    depth_time_stamps = zeros(N, 1);
    rgb_filenames     = strings(N, 1);
    depth_filenames   = strings(N, 1);
    GT_rotations      = zeros(3, 3, N);
    GT_translations   = zeros(3, N);

    for i = 1:length(rgb_depth_alignment_data)
        single_string = rgb_depth_alignment_data{i};
        splitted = split(single_string, ' ');
        rgb_time_stamps(i,1)    = str2num(splitted{1});
        rgb_filenames(i,1)      = strcat(dataset_root_path, seq_path, splitted{2});
        depth_time_stamps(i,1)  = str2num(splitted{3});
        depth_filenames(i,1)    = strcat(dataset_root_path, seq_path, splitted{4});

        %> find the associate GT to the RGB image
        [val, index] = min(abs(rgb_time_stamps(i,1) - GT_time_stamps));
        R = GT_quaternions_data(index, :);
        GT_rotations(:,:,i) = quat2rotm(R([4 1 2 3]));
        T = GT_translations_data(index, :);
        GT_translations(:,i) = T';
    end

end