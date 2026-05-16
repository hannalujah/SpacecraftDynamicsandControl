clc;
clear;
close all;

%% =========================================================
%  Mars Mission Geometry Visualization
% ==========================================================

figure('Color','w');
hold on;
axis equal;
grid on;

view(135,20);

%% =========================================================
%  Constants
% ==========================================================

R_mars = 3396.19;          % km

r_LMO = R_mars + 400;      % km
r_GMO = 20424.2;           % km

%% =========================================================
%  Textured Mars Sphere
% ==========================================================

% Put mars_texture.jpg inside your MATLAB folder
mars_texture = imread('mars_texture.jpg');

[Xm,Ym,Zm] = sphere(300);

surface(R_mars*Xm,...
        R_mars*Ym,...
        R_mars*Zm,...
        'FaceColor','texturemap',...
        'CData',mars_texture,...
        'EdgeColor','none');

lighting gouraud
material dull

light('Position',[1 0 1],...
      'Style','infinite');

%% =========================================================
%  Darkened Hemisphere (Shadow Side)
% ==========================================================

[Xs,Ys,Zs] = sphere(300);

% Dark side along negative n2 direction
Ys(Ys > 0) = NaN;

surf(R_mars*Xs,...
     R_mars*Ys,...
     R_mars*Zs,...
     'FaceColor','k',...
     'EdgeColor','none',...
     'FaceAlpha',0.30);

%% =========================================================
%  Mars Shadow Cylinder
% ==========================================================

Rcyl = R_mars;
Lcyl = 18000;

[XC,YC,ZC] = cylinder(Rcyl,120);

ZC = ZC * Lcyl;

% Cylinder aligned with -n2 direction
surf(XC,...
     -ZC,...
      YC,...
     'FaceAlpha',0.10,...
     'EdgeColor','none',...
     'FaceColor',[0.6 0.6 0.6]);

%% =========================================================
%  LMO Orbit
% ==========================================================

theta = linspace(0,2*pi,700);

inc = deg2rad(30);

x_lmo = r_LMO*cos(theta);
y_lmo = r_LMO*sin(theta)*cos(inc);
z_lmo = r_LMO*sin(theta)*sin(inc);

plot3(x_lmo,...
      y_lmo,...
      z_lmo,...
      'Color',[0.35 0.5 1],...
      'LineWidth',3);

%% =========================================================
%  GMO Orbit (Equatorial)
% ==========================================================

theta_gmo = linspace(0,2*pi,700);

x_gmo = r_GMO*cos(theta_gmo);
y_gmo = r_GMO*sin(theta_gmo);
z_gmo = zeros(size(theta_gmo));

plot3(x_gmo,...
      y_gmo,...
      z_gmo,...
      'Color',[0.75 0.55 0.2],...
      'LineWidth',3);

%% =========================================================
%  Initial Spacecraft Positions
% ==========================================================

%-------------------------
% LMO Initial Position
%-------------------------

theta0_lmo = deg2rad(60);

x0_lmo = r_LMO*cos(theta0_lmo);
y0_lmo = r_LMO*sin(theta0_lmo)*cos(inc);
z0_lmo = r_LMO*sin(theta0_lmo)*sin(inc);

scatter3(x0_lmo,...
         y0_lmo,...
         z0_lmo,...
         120,...
         'b',...
         'filled');

%-------------------------
% GMO Initial Position
%-------------------------

theta0_gmo = deg2rad(250);

x0_gmo = r_GMO*cos(theta0_gmo);
y0_gmo = r_GMO*sin(theta0_gmo);
z0_gmo = 0;

scatter3(x0_gmo,...
         y0_gmo,...
         z0_gmo,...
         120,...
         [0.35 0.2 0],...
         'filled');

%% =========================================================
%  Inertial Axes
% ==========================================================

Laxis = 7000;

quiver3(0,0,0,...
        Laxis,0,0,...
        'k',...
        'LineWidth',2,...
        'MaxHeadSize',0.5);

quiver3(0,0,0,...
        0,Laxis,0,...
        'k',...
        'LineWidth',2,...
        'MaxHeadSize',0.5);

quiver3(0,0,0,...
        0,0,Laxis,...
        'k',...
        'LineWidth',2,...
        'MaxHeadSize',0.5);

%% =========================================================
%  Axis Labels
% ==========================================================

xlabel('$n_1$ [km]',...
       'Interpreter','latex',...
       'FontSize',18);

ylabel('$n_2$ [km]',...
       'Interpreter','latex',...
       'FontSize',18);

zlabel('$n_3$ [km]',...
       'Interpreter','latex',...
       'FontSize',18);

%% =========================================================
%  Plot Appearance
% ==========================================================

set(gca,...
    'FontSize',15,...
    'LineWidth',1.2);

xlim([-25000 25000]);
ylim([-25000 25000]);
zlim([-9000 9000]);

camproj perspective
box on

rotate3d on