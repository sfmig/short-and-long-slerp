%% Slerp interpolation short way vs long way demo
%
% Demo for the function 'compute_slerp_path_forcing_long_or_short'
%
% Author: S Minano, Oxford Flight Group (University of Oxford)
% Date: 29-Jan-2021; 
% Last revision: 29-Jan-2021
% Matlab version: 9.9.0.1538559 (R2020b) Update 3
% Copyright (c) 2021, Author

%% Compute short and long slerp path, from q0 to q1

% Initial and end points 
% we take a rotation of 50deg around z-axis
q0 = axang2quat([0 0 1 0]);
q0 = q0./vecnorm(q0,2,2);
q1 = axang2quat([0 0 1 50]);
q1 = q1./vecnorm(q1,2,2);

% t-parameter
n_steps = 10;
t_param = linspace(0,1,n_steps);

% Compute quaternions assuming long and short path
q_interp_short = compute_slerp_path_forcing_long_or_short(q0,q1,t_param,'short')';
q_interp_long = compute_slerp_path_forcing_long_or_short(q0,q1,t_param,'long')';


%% Plot each path on the quaternion unit sphere 
% plot as a 3d problem, i.e., project on the second coord (0) (q = q_r + q_i*i + q_j*j + q_k*k)
q0_proj = q0([1 3 4]);
q1_proj = q1([1 3 4]);

q_interp_short_proj = q_interp_short(:,[1 3 4]);
q_interp_long_proj = q_interp_long(:,[1 3 4]);

fig_h = figure;
subplot(2,3,[1 4])
quiver3(0,0,0,...
        q0_proj(1),q0_proj(2),q0_proj(3),1,'r',...
        'DisplayName','q0 (projected)');grid on; hold on
quiver3(0,0,0,...
        q1_proj(1),q1_proj(2),q1_proj(3),1,'b',...
        'DisplayName','q1 (projected)');grid on; hold on
quiver3(0,0,0,...
        -q1_proj(1),-q1_proj(2),-q1_proj(3),1,'b:',...
        'DisplayName','-q1 (projected)');grid on; hold on
% start and end pts
scatter3(q0_proj(:,1),...
         q0_proj(:,2),...
         q0_proj(:,3),...
         10,'r','filled',...
         'DisplayName','start point');hold on
scatter3(q1_proj(:,1),...
         q1_proj(:,2),...
         q1_proj(:,3),...
         10,'b','filled',...
         'DisplayName','end point');hold on
% short way
scatter3(q_interp_short_proj(:,1),...
         q_interp_short_proj(:,2),...
         q_interp_short_proj(:,3),...
         10,'m','DisplayName','short way');hold on
axis equal
% long way
scatter3(q_interp_long_proj(:,1),...
         q_interp_long_proj(:,2),...
         q_interp_long_proj(:,3),...
         10,'c','DisplayName','long way');hold on
     
axis equal
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
legend
xlabel('q_r');ylabel('q_j');zlabel('q_k')
legend('Location','Best')
title('quaternion unit hypersphere (projected on 3D)')

%% Rotate pose
% plot a reference frame rotating from initial to end point, following the short and the long slerp path

figure(fig_h);
ijk = eye(3);
translation_xyz = [linspace(0,n_steps,size(q_interp_short,1))' zeros(size(q_interp_short,1),1) zeros(size(q_interp_short,1),1)];
colors_ijk = {'r','g','b'};
axis_str = {'x', 'y', 'z'};

q_interp_short_and_long = {q_interp_short, q_interp_long};
q_interp_title_str = {'short way (magenta)','long way (cyan)'};

subplt_c = [2 3];
for c_i=1:length(q_interp_short_and_long)
    subplot(2,3,subplt_c)
    for kk=1:size(ijk,1)
        versor = ijk(kk,:);
        versor_rot_all_frames = rotatepoint(quaternion(q_interp_short_and_long{c_i}),...
                                            versor); 
        quiver3(translation_xyz(:,1),translation_xyz(:,2),translation_xyz(:,3),...
                versor_rot_all_frames(:,1),...
                versor_rot_all_frames(:,2),...
                versor_rot_all_frames(:,3),...
                0.15,...
                colors_ijk{kk},...
                'LineWidth',1.5,...
                'DisplayName',axis_str{kk});hold on
        title(q_interp_title_str{c_i})
        axis equal
        view(2)
        xlabel('rotation step');ylabel('y');zlabel('z')
        set(gca, 'YTickLabel', [])
        ylim([-1 1])
        view(3)
    end
    
    subplt_c = [5 6];
end



