
%> Plot cumulative histogram
close all;
source_repository_path = "/home/chchien/BrownU/research/RGBD_1-Point_RANSAC/Sourav_Code/";

method_Names = ["classic"; "2Loops"; "3Loops"];

path_classic = strcat(source_repository_path, "Methods_Profiling/pose_estimation_errs/pose_estimation_err_classic.mat");
path_2Loops  = strcat(source_repository_path, "Methods_Profiling/pose_estimation_errs/pose_estimation_err_2Loops.mat");
path_3Loops  = strcat(source_repository_path, "Methods_Profiling/pose_estimation_errs/pose_estimation_err_3Loops.mat");

%> Classic method
RANSAC_Iterations_classic_path = load(path_classic).pose_estimation_err_classic;
% h = histogram(RANSAC_Iterations_classic_path(:,1), 'Normalization', 'cdf');
h = histogram(RANSAC_Iterations_classic_path(:,1), ...
              max(RANSAC_Iterations_classic_path(:,1)), 'Normalization', 'cdf');
%> Display by curves
figure;
iter_ = round(h.BinEdges(1,1:end-1));
prob_ = h.Values;
plot(iter_, prob_, "b.-", 'DisplayName', '60%~69% outlier ratio', 'LineWidth',2);
xlim([0, max(RANSAC_Iterations_classic_path(:,1))]);
ylim([0, 1]);
xlabel('Number of RANSAC iterations');
ylabel('Probaiblity');
lg = legend;
lg.FontSize = 15;
set(gca, 'FontSize', 15);
set(gcf,'color','w');

%> 2-Loop method
RANSAC_Iterations_2Loop_path = load(path_2Loops).pose_estimation_err_2Loops;
fail_indices = find(RANSAC_Iterations_2Loop_path(:,6) == 0);
RANSAC_Iterations_2Loop_path(fail_indices, :) = [];
h_2Loop_iter1 = histogram(RANSAC_Iterations_2Loop_path(:,1), ...
                    max(RANSAC_Iterations_2Loop_path(:,1)), 'Normalization', 'cdf');
hold on;
h_2Loop_iter2 = histogram(RANSAC_Iterations_2Loop_path(:,2), ...
                    max(RANSAC_Iterations_2Loop_path(:,2)), 'Normalization', 'cdf');
hold off;
%> Display by curves
figure;
iter_1 = round(h_2Loop_iter1.BinEdges(1,1:end-1));
prob_1 = h_2Loop_iter1.Values;
iter_2 = round(h_2Loop_iter2.BinEdges(1,1:end-1));
prob_2 = h_2Loop_iter2.Values;
plot(iter_1, prob_1, "b.-", 'DisplayName', '1st Loop, 60%~69% outlier ratio', 'LineWidth',2);
hold on;
plot(iter_2, prob_2, "g.-", 'DisplayName', '2nd Loop, 60%~69% outlier ratio', 'LineWidth',2);
hold off;
xlim([0, max(max(RANSAC_Iterations_2Loop_path(:,1)), max(RANSAC_Iterations_2Loop_path(:,2)))]);
ylim([0, 1]);
xlabel('Number of RANSAC iterations');
ylabel('Probaiblity');
lg = legend;
lg.FontSize = 15;
set(gca, 'FontSize', 15);
set(gcf,'color','w');
pause(1);

%> 3-Loop method
RANSAC_Iterations_3Loop_path = load(path_3Loops).pose_estimation_err_3Loops;
fail_indices = find(RANSAC_Iterations_3Loop_path(:,7) == 0);
RANSAC_Iterations_3Loop_path(fail_indices, :) = [];
h_3Loop_iter1 = histogram(RANSAC_Iterations_3Loop_path(:,1), ...
                    max(RANSAC_Iterations_3Loop_path(:,1)), 'Normalization', 'cdf');
hold on;
h_3Loop_iter2 = histogram(RANSAC_Iterations_3Loop_path(:,2), ...
                    max(RANSAC_Iterations_3Loop_path(:,2)), 'Normalization', 'cdf');
hold on;
h_3Loop_iter3 = histogram(RANSAC_Iterations_3Loop_path(:,3), ...
                    max(RANSAC_Iterations_3Loop_path(:,3)), 'Normalization', 'cdf');
hold off;
                
%> Display by curves
pause(0.5);
figure;
iter_1 = round(h_3Loop_iter1.BinEdges(1,1:end-1));
prob_1 = h_3Loop_iter1.Values;
iter_2 = round(h_3Loop_iter2.BinEdges(1,1:end-1));
prob_2 = h_3Loop_iter2.Values;
iter_3 = round(h_3Loop_iter3.BinEdges(1,1:end-1));
prob_3 = h_3Loop_iter3.Values;
plot(iter_1, prob_1, "b.-", 'DisplayName', '1st Loop, 60%~69% outlier ratio', 'LineWidth',2);
hold on;
plot(iter_2, prob_2, "g.-", 'DisplayName', '2nd Loop, 60%~69% outlier ratio', 'LineWidth',2);
hold on;
plot(iter_3, prob_3, "r.-", 'DisplayName', '3rd Loop, 60%~69% outlier ratio', 'LineWidth',2);
xlim([0, max(max(max(RANSAC_Iterations_3Loop_path(:,1)), max(RANSAC_Iterations_3Loop_path(:,2))), max(RANSAC_Iterations_3Loop_path(:,3)))]);
ylim([0, 1]);
xlabel('Number of RANSAC iterations');
ylabel('Probaiblity');
lg = legend;
lg.FontSize = 15;
set(gca, 'FontSize', 15);
set(gcf,'color','w');