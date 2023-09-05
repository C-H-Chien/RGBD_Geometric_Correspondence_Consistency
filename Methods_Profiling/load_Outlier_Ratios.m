
all_Outlier_Ratios_Dir = dir("/home/chchien/BrownU/research/RGBD_1-Point_RANSAC/Sourav_Code/Methods_Profiling/Outlier_Ratios/*.mat");

Outlier_Ratio_Ranges = cell(10,1);

%> Loop over all outlier ratio .mat files
for q = 1:length(all_Outlier_Ratios_Dir) 
    mat_FileName = all_Outlier_Ratios_Dir(q).name;
    imgPairs_Outlier_Ratios_per_Seq = load(mat_FileName).all_Outlier_Ratios;
    
    %> For each .mat file, store data to a cell array according to the
    %  outlier ratio value
    for p = 1:size(imgPairs_Outlier_Ratios_per_Seq, 1)
        outlier_ratio = double(imgPairs_Outlier_Ratios_per_Seq(p,4));
        if outlier_ratio == 1, continue; end
        cell_array_category = floor((outlier_ratio*100)/10)+1;
        Outlier_Ratio_Ranges{cell_array_category, 1} = ...
            [Outlier_Ratio_Ranges{cell_array_category, 1}; imgPairs_Outlier_Ratios_per_Seq(p,:)];
    end
end
