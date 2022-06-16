function p_interp = compute_slerp_path_forcing_long_or_short(p0,p1,t,...
                                                              path_str,...
                                                              DOT_THRESHOLD)
%% Computes slerp path between two quaternions, with the option of going the short way or the long way
%  Computes the slerp interpolated path between two quaternions p0 and p1. 
%  Slerp interpolation guarantees a constant angular velocity rotation from the start quaternion to the end
%  If quaternions are too close to each other, linear interpolation is used instead.
%
%  Inputs:
%     p0 - 1x4 double representing quaternion starting point
%     p1 - 1x4 double representing quaternion end point
%     t - 1xN double representing parameter t, interpolation coefficient from 0 to 1 (inclusive). At t=0, p_interp = p0_unit; at t=1, p_interp=p1_unit;
%     path_str - [OPTIONAL, default = 'short'] a string, either 'short' or 'long', indicating whether we want to compute the slerp path the 'short way' or the 'long way'. 
%                 If not specified, the short way is computed. For more info about short vs long way, see https://en.wikipedia.org/wiki/Slerp > Quaternion Slerp section.
%     DOT_THRESHOLD - [OPTIONAL, default = 0.9995]. If the absolute value of the dot product between p0 and p1 is above this threshold, the quaternions are considered almost parallel and a linear interpolation is computed instead
%
%  Outputs:
%     p_interp - Nx4 double, each row representing a normalised quaternion in the interpolated path. 
%                Start and end points are included: p_interp(1,:) = p0_unit; p_interp(end,:) = p1_unit;
%
% Based on code snippets from:
% - https://en.wikipedia.org/wiki/Slerp 
% - https://stackoverflow.com/questions/48331762/interpolating-curve-in-spherical-coordinates/48332044
%
%
%  Author: S Minano, Oxford Flight Group (University of Oxford)
% Date: 29-Jan-2021; 
% Last revision: 29-Jan-2021
% Matlab version: 9.9.0.1538559 (R2020b) Update 3
% Copyright (c) 2021, S Minano 
% All rights reserved.

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Arguments checking
    arguments
            p0 (1,4) double {mustBeNumeric, mustBeFinite}
            p1 (1,4) double {mustBeNumeric, mustBeFinite}
            t (1,:) double {mustBeNumeric, mustBeFinite} %MATLAB row-column conversion applies so that a size specified as (1,:) can accept a size of 1-by-n and n-by-1
            path_str (1,:) char {mustBeMember(path_str,{'short','long'})} = 'short' %because we specify a default argument, input is optional
            DOT_THRESHOLD (1,1) double = 0.9995
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute dot product between normalised quats
    % Compute normalised quaternions
    p0_unit = p0(:) / norm(p0);
    p1_unit = p1(:) / norm(p1);

    % Compute dot product between quaternions
    % omega: angle subtended by the arc between the quaternions; dot = cos(omega)
    dot_p = dot(p0_unit,p1_unit);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Force either short or long path
    % - if dot_p>0: slerp will take short path
    % - if dot_p<0: slerp will take long path
    % check which case occurs and fix depending on desired path
    
    % Force appropriate path
    if or(and(strcmp(path_str,'short'),...
              dot_p < 0),... % if SHORT path required and dot_p<0 (i.e., slerp would take long path) : flip sign of end point and update dot product
          and(strcmp(path_str,'long'),...
               dot_p > 0))  % if LONG path required and dot_p>0 (i.e., slerp would take short path) : flip sign of end point and update dot product   
       p1_unit = - p1_unit;
       dot_p = - dot_p;     
    end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute slerp path
    
    % If abs(dot product) within machine epsilon of 1: use linear interpolation 
    if abs(dot_p) > DOT_THRESHOLD
        scale_0 = 1-t;
        scale_1 = t;

    % Else: apply scales for slerp    
    else
        omega = acos(dot_p);
        scale_0 = (sin((1.0-t)*omega)/sin(omega));
        scale_1 = (sin(t*omega)/sin(omega));  
    end
    
    % compute linear combination
    p_interp = bsxfun(@times,scale_0,p0_unit) + bsxfun(@times,scale_1,p1_unit); %scale_0.*p0_unit + scale_1.*p0_unit;
end
