function [filt_sig] = filter_2d(sig, filt_fun_2d, sigma, n)
% function [filt_sig, dx] = filter_2d(sig, filt_fun_2d, sigma, n)
%
% Filter the signal at pixel locations using first-order Gaussian
% derivative filter
%
% Parameters:
%    sig:          original image to be filtered
%    filt_fun_2d:  a 2d filter kernel that can be resampled
%    n:            2^n number of subdivions within the pixel (NOT USED IN THE CURRENT VERSION) 
%
% Output:
%   filt_sig : resulting response signal 
%
% (c) LEMS, Brown University
% Chiang-Heng Chien (chiang-heng_chien@brown.edu)
% March 2022

% compute the filter at this shift 
filt_resamp = filt_fun_2d(sigma, 0, 0);

% filter sig with this filter
filt_sig = imfilter(sig, filt_resamp, 'conv','same');

end
