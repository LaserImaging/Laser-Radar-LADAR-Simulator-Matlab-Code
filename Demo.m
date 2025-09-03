%% ------------------ [LADAR Simulator Demonstration] ------------------ %%
% This file demonstrates the simulator's capability to simulate the direct 
% detection time-of-flight LADAR systems and to produce 3D simulated 
% scanning images under a wide variety of conditions. It reads the 
% [faces & vertices] 3D information for the CAD model from the 
% "3D CAD Model.mat" file, scanning it according to the scanning parameters
% using the "LADAR.m" function, and displays the results in two different 
% formats: spherical image and 3D point cloud.

% Copyright (c) 2025 holder: 
% Ali A. Al-Temeemy is a Professor in the Department of Laser and 
% Optoelectronics Engineering at Al-Nahrain University and an honorary 
% research fellow in the Department of Electrical Engineering and 
% Electronics at the University of Liverpool.

% In return for making this code available, I would appreciate that you 
% cite the following publications:

% [1]
% Ali Adnan Al-Temeemy; "The Development of a 3D LADAR Simulator Based on 
% a Fast Target Impulse Response Generation Approach," 3D Research, Springer
% (2017), volume 8, 31, Aug 2017, ISSN2092-6731, doi:10.1007/s13319-017-0142-y.

% [2]
% Ali A. Al-Temeemy and J. W. Spencer. "Simulation of 3D Ladar Imaging System
% using Fast Target Response Generation Approach," In Proceeding of SPIE - 
% Optical Design and Engineering IV, Marseille, France,  volume 8167, pages
% 816720-(1-9), Sep 2011, doi: 10.1117/12.902309.
%% --------------------------------------------------------------------- %%

%% ----------------------- 3D CAD Model Reading ------------------------ %%
clear;close;clc; % Initialization.
load('3D CAD Model','Obj'); % Load 3D Model FV format.
F=Obj.F;V=Obj.V; %  [Faces & Vetices] 3D Object Data.
%% --------------------------------------------------------------------- %%

%% ----------------------- Display 3D CAD Model ------------------------ %%
Oc=[.6; .2 ;.1]*ones(1,length(V));Oc=Oc';figure; % Model.
patch('Vertices',V,'Faces',F,'FaceVertexCData',Oc,'FaceColor','interp',...
'FaceLighting','Phong','EdgeColor','none','SpecularColorReflectance',0.9);
light;grid on;xlabel('X_O [m]');ylabel('Y_O [m]');zlabel('Z_O [m]');
view(-43,54);lighting gouraud; % Figure setup.
%% --------------------------------------------------------------------- %%

%% ----------------------- 3D CAD Model Scanning ----------------------- %%
% --[Set Ladar Scanning Parameters]
SRA=0.2;       % SR: Scanning Angular Resolution.                       (*)
SRE=0.2;       % SR: Scanning Elevation Resolution.                     (*)
% THETA: Angular Angle.                  
THETAI=15;    % THETAI: Initial Angular Angle Value                    (*)
THETAF=-15;     % THETAF: Final Angular Angle Value.                     (*)
% PHI: Elevation Angle.
PHII=10;     % PHII: Initial Elevation Angle Value.                    (*)
PHIF=-10;      % PHIF: Final Elevation Angle Value.                      (*)
% --[Apply LADAR Function to scan Model]
[SphLIM,CarLIM]=LADAR(SRA,SRE,THETAI,THETAF,PHII,PHIF,V,F);
%% --------------------------------------------------------------------- %%

%% ------------------- [Display LADAR Scanning Data] ------------------- %%
% --[Spherical Ladar Range Image]
figure;imagesc([SphLIM(1,1,1)*180/pi; SphLIM(1,end,1)*180/pi],...
[SphLIM(1,1,2)*180/pi SphLIM(end,1,2)*180/pi],SphLIM(:,:,3));
set(gca,'XDir','reverse','YDir','normal','color','black','xcolor',...
    'red','ycolor','blue');
grid on;xlabel('Pan Angle');ylabel('Tilt Angle'); % Figure setup.
axis equal;tag=colorbar('peer',gca);set(get(tag,'ylabel'),...
'String','Distance','color','r');set(tag,'ycolor','r');% Figure setup.
% --[Ladar 3D Points Cloud Image]
figure;plot3(CarLIM(:,:,1),CarLIM(:,:,2),CarLIM(:,:,3),'.r');light;
set(gca,'color','black','xcolor','red','ycolor','green','zcolor','blue');
axis equal;grid on;xlabel('X');ylabel('Y');zlabel('Z'); % Figure setup.
%% --------------------------------------------------------------------- %%