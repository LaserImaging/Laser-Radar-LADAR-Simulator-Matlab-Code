%%%%%%%%%%--------------Ladar Simulator Function----------------%%%%%%%%%%%
% This function implements the LADAR scanning Process based on the
% 'Fast Target Impulse Response Generation Approach'.

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


function [SphLIM,CarLIM]=LADAR(SRA,SRE,THETAI,THETAF,PHII,PHIF,V,F) %  
%%%%%%%%%%--------------Ladar Simulator Program-----------------%%%%%%%%%%%
% Outputs.
% SphLIM: Spherical Image.
% CarLIM: Cartesian Image.
%%%%%%%%%%--------------------Ladar Parameters------------------%%%%%%%%%%%
% Set Ladar Scanning Parameters.
LR=1000;              % Laser Line Lenght.                              (*)
SRA=SRA*pi/180;       % SR: Scanning Angular Resolution.                (*)
SRE=SRE*pi/180;       % SR: Scanning Elevation Resolution.              (*)
% THETA: Angular Angle.                  
THETAI=THETAI*pi/180; % THETAI: Initial Angular Angle Value             (*)
THETAF=THETAF*pi/180; % THETAF: Final Angular Angle Value.              (*)
% PHI: Elevation Angle.
PHII=PHII*pi/180;     % PHII: Initial Elevation Angle Value.            (*)
PHIF=PHIF*pi/180;     % PHIF: Final Elevation Angle Value.              (*)
%%%%%%%%%%------------------------------------------------------%%%%%%%%%%%
%%%%%%%%%%------------------Ray Tracing Loop--------------------%%%%%%%%%%%
% Calculate the Initial Parameters for Laser Collision Points.
[V0,v0,v1,Tn,n,d00,d01,d11,ID,AE,THL,PHL,l,Dis]=param(F,V,SRA,SRE,THETAI,THETAF,PHII,PHIF,LR); % Calculate the parameters.
%-------------------------------------------------------------------%
for i=1:size(F,1) % Loop for Laser Point.
[Dis]=Intpoint(V0(:,i),v0(:,i),v1(:,i),Tn(i),d00(i),d01(i),d11(i),ID(i),l,n(:,i),AE(:,i),THL,PHL,Dis); % Calculate Collision Points.
end
%------------------------------------------------------------------%
% Calculate Ladar Image in Cartesian.(With respect to the Ladar Position)
Dis(Dis==LR)=nan;CarLIM=l(:,:,1:3).*repmat(Dis,[1 1 3]);
% Calculate Ladar Image in Spherical.(With respect to the Ladar Position)
[theta,phi,lr] = cart2sph(CarLIM(:,:,1),CarLIM(:,:,2),CarLIM(:,:,3));
SphLIM=cat(3,theta,phi,lr);SphLIM(1,:,1)=THL;SphLIM(:,1,2)=PHL; % Reshape Matrix.
%%%%%%%%%%------------------------------------------------------%%%%%%%%%%%
return

% This function calculate the CAD object parameters.
function [V0,v0,v1,Tn,n,d00,d01,d11,ID,AE,THL,PHL,l,Dis]=param(F,V,SRA,SRE,THETAI,THETAF,PHII,PHIF,LR)
%% Inputs
% F: Object Faces.
% V: Object Vertices.
% SRA & SRE: Azmith & Elevation Angular Resolution.   
% THETAI & THETAF : Initial & Final Azmith Angles. 
% PHII & PHIF: Initial & Final Elevation Angles.
% LR: Laser Line Lenght.  
%% Outputs
% V0: Ref. Vertices.
% v0: V2-V0.
% v1: V1-V0.
% ITn: 1/ dot(N,(V0-P0)).
% n: Object Normal.
% d00: dot(v0, v0);
% d01: dot(v0, v1);
% d11: dot(v1, v1);
% ID: 1 / (d00 * d11 - d01 * d01);
% AE: Azmith and Elevation Faces Angles.
% THL & PHL: Azmith & Elevation Vectors.
% l & Dis: Ladar Scan Vectors and Distances
% div: Number of Border Points.
%% Calulate the parameters
F=F';V=V'; % Prep.
V0=V(:,F(1,:)); % Ref. Vertices.
V1=V(:,F(2,:)); % Second Vertices.
V2=V(:,F(3,:)); % Third Vertices.
v0=V2-V0; % v0.
v1=V1-V0; % v1.
n=cross(v1,v0);n=n./repmat(sqrt(sum(n.^2)),3,1); % v1xv0.
PV=V0;% PV.
Tn= dot(n,PV); % Time (numenator).
d00= dot(v0, v0);
d01= dot(v0, v1);
d11= dot(v1, v1);
ID=  1./ ((d00 .* d11) - (d01 .* d01));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Faces FOV using Round Points.
%--------Azmith and Elevation angles for the Faces--------%
x1=cat(1,V0(1,:),V1(1,:),V2(1,:));y1=cat(1,V0(2,:),V1(2,:),V2(2,:));
z1=cat(1,V0(3,:),V1(3,:),V2(3,:));x2=cat(1,V1(1,:),V2(1,:),V0(1,:));
y2=cat(1,V1(2,:),V2(2,:),V0(2,:));z2=cat(1,V1(3,:),V2(3,:),V0(3,:));
%%%%--------------------Calculate Round Point-------------------%%%%
tp=(-z1.*x1.*x2)-(z1.*y1.*y2)+(z2.*x1.^2)+(z2.*y1.^2)./(z1.*x2.^2)+...
(z1.*y2.^2)+(z2.*x1.^2)+(z2.*y1.^2)-(z1.*x1.*x2)-(z1.*y1.*y2)-(z2.*x1.*x2)-(z2.*y1.*y2);
tp(tp>1)=nan;tp(tp<0)=nan; % Filter the value not between (0,1).
%%%%------------------------------------------------------------%%%%
ph1=atan((z1+tp.*(z2-z1))./(sqrt((x1+tp.*(x2-x1)).^2+(y1+tp.*(y2-y1)).^2)));
[Th,ph2,~]=cart2sph(x1,y1,z1); % Calculate Azim. & Elev. for the Vertices.
% Select the Field of View (Azmith & Elevation) for each Object Face.
AE=[min(Th);max(Th);min([ph1;ph2]);max([ph1;ph2])]; % Azmith and Elevation Angles.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------Ladar Scan Vectors and Distances-------------%(Constant PHI)
THL=THETAI:-SRA:THETAF;PHL=PHII:-SRE:PHIF;[THETA,PHI,R]=meshgrid(THL,PHL,LR);
[x,y,z] = sph2cart(THETA,PHI,R);l=cat(3,x,y,z);Dis=LR*ones(size(THETA));
%-------------------------------------------------------------------------%
return

% Calculate the point of intersection with the patch surface
function [Dis]=Intpoint(V0,v0,v1,Tn,d00,d01,d11,ID,l,n,AE,THL,PHL,Dis)
%% Inputs
% V0: Ref. Vertices.
% v0: V2-V0.
% v1: V1-V0.
% ITn: 1/ dot(N,(V0-P0)).
% d00: dot(v0, v0);
% d01: dot(v0, v1);
% d11: dot(v1, v1);
% ID: 1 / (d00 * d11 - d01 * d01);
% l: Line vector.
% n: Object Normal.
% AE: Azmith and Elevation Faces Angles.
% THL & PHL: Azmith & Elevation Angles.
% Dis: Distance of intersection point Matrix with Line vectors.
%% Outputs
% Dis: Distance of intersection point Matrix with Line vectors.
%% Find The Laser Beam Vectors That Hit the Face.
if AE(1) < -1.5708 || AE(2) > 1.5708; return;end % If Object Behind Ladar.
row=find(PHL<=AE(4),1,'first'):find(PHL>=AE(3),1,'last');
if isempty(row);return;end % If Empty Return.
col=find(THL<=AE(2),1,'first'):find(THL>=AE(1),1,'last');
if isempty(col);return;end % If Empty Return.
Lrow=length(row);Lcol=length(col);L=Lrow*Lcol;
l=reshape(l(row,col,:),L,3);di=reshape(Dis(row,col),L,1);
%% Calulate the Intersection Point
Dist=Tn./(l*n); % Distance of intersection point.
IntP=(l.*repmat(Dist,1,3))';IntP(IntP==inf)=nan; % Intersection points.
v2=(IntP-repmat(V0,1,L)); % v2.
% Compute barycentric coordinates
d02= v0'*v2;
d12= v1'*v2;
u = (d11 .* d02 - d01 .* d12) .* ID;
v = (d00 .* d12 - d01 .* d02) .* ID;
%Check if point is in triangle
u(u<0)=nan;v(v<0)=nan;uv=u+v;uv(uv>1)=nan;uv(uv>=0)=1;
Dist=Dist.*uv'; % Vectors Distance matrix.
cond=Dist< di;di(cond)=Dist(cond); % Check for Minimum Distance.
Dis(row,col)=reshape(di,[Lrow Lcol]); % Replace the with the Minimum Distance.
return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

