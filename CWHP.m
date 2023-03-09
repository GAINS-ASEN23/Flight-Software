clc; close all; clear all;

%Declared Constants:
mu_earth = 3.986004418e14;      %Earth Graitational Constant
rad_earth = 6378.1363;          % km
mu_moon = 4.9048695e12;         % Moon Graitational Constant
a_moon = 1.74e6+5e4;            % Semimajor axis of Moon's orbit around Earth [m]

%Initial Circular Radius of Space Shuttle
r_int = 500;

% Create initial coordinates representing the target (Moon)
   xt = a_moon;
   yt = 0;
   zt = 0;
xdott = 0;
ydott = sqrt(mu_earth/(a_moon));
zdott = 0;
rT = [xt, yt, zt]';
vT = [xdott,ydott,zdott]';

% Add inclination to the orbit
inc = 5.145;                    % Degrees
RYM = [cosd(inc) 0 sind(inc); 0 1 0; -sind(inc) 0 cosd(inc)];
rT = RYM*rT;
vT = RYM*vT;

% Create initial coordinates representing the chaser satellite 
   xi = rT(1) + 1000;
   yi = rT(2);
   zi = rT(3);
xdoti = vT(1);
ydoti = vT(2);
zdoti = vT(3);
rI = [xi, yi, zi]';
vI = [xdoti,ydoti,zdoti]';

% Specify the time interval of interest in seconds
t = 0:60:400*60;

% Convert the chaser satellite ECI coordinates into the Hill frame of
% reference
[rHill,vHill] = ECI2Hill_Vectorized(rT,vT,rI,vI);

% Add initial displacement on the x vector
rHill(1) = rHill(1)+1;

% Now ... recompute the chaser satellite vector ...
[rI,vI] = Hill2ECI_Vectorized(rT,vT,rHill,vHill);

% Find the angular rate of the target orbit (shuttle)
omega = sqrt(mu_earth/sqrt(sum(rT.^2))^3);

% Use the Linear Propagator (LOP) to propagate Hill's coordinates forward in time
% This is to find the relative orbital motion of the "Chaser" satellite
[rHill,vHill] = CWHPropagator(rHill,vHill,omega,t);


figure;
plot(rHill(1,:), rHill(2,:))



% function bruh()
% %% Purpose:
% % This is a demo of how to use the CW equations to propagate an orbit
% % using the relative positions of a target satellite and a chaser
% % satellite.  The frame of reference is the target satellite orbit
% % (Shuttle).  We are interested in examining the motion of the Hubble
% % telescope relative to the orbit of the shuttle.
% %
% % Programmed by Darin C Koblick 12/01/2012
% 
% %% Begin Code Sequence
% %Declared Constants:
%       mu = 4.9048695e12;     %Earth Graitational Constant
% earthRad = 1737.4;       %km
% %Initial Circular Radius of Space Shuttle
% r_int = 500;
% %Create initial coordinates representing the target (shuttle)
%    xt = earthRad + r_int;
%    yt = 0;
%    zt = 0;
% xdott = 0;
% ydott = sqrt(mu/(earthRad+r_int));
% zdott = 0;
% rT = [xt, yt, zt]';
% vT = [xdott,ydott,zdott]';
% %Add 51.65 degrees of inclination to the orbit
% inc = -51.65;
% RYM = [cosd(inc) 0 sind(inc); 0 1 0; -sind(inc) 0 cosd(inc)];
% rT = RYM*rT;
% vT = RYM*vT;
% %Create initial coordinates representing the chaser satellite (Hubble)
% %Initial Positions and Velocities of the hubble telescope from the space 
% %shuttle bay All units are in km/s
% %Assume that both telescope and shuttle are docked initially
%    xi = rT(1);
%    yi = rT(2);
%    zi = rT(3);
% xdoti = vT(1);
% ydoti = vT(2);
% zdoti = vT(3);
% rI = [xi, yi, zi]';
% vI = [xdoti,ydoti,zdoti]';
% %Specify the time interval of interest in seconds
% t = 0:60:400*60;
% %Convert the chaser satellite ECI coordinates into the Hill frame of
% %reference
% [rHill,vHill] = ECI2Hill_Vectorized(rT,vT,rI,vI);
% %Add initial displacement on the x vector
% rHill(1) = rHill(1)+1;
% %Now ... recompute the chaser satellite vector ...
% [rI,vI] = Hill2ECI_Vectorized(rT,vT,rHill,vHill);
% %Find the angular rate of the target orbit (shuttle)
% omega = sqrt(mu/sqrt(sum(rT.^2))^3);
% %Use the Linear Propagator (LOP) to propagate Hill's coordinates forward in time
% %This is to find the relative orbital motion of the "Chaser" satellite
% [rHill,vHill] = CWHPropagator(rHill,vHill,omega,t);
% 
% 
% %Use a nonlinear propagator to determine the Target tragectory as well as
% %the Chaser tragectory
%     [rTgt,vTgt] = keplerUniversal(repmat(rT,[1 length(t)]),repmat(vT,[1 length(t)]),t,mu);
% [rChase,vChase] = keplerUniversal(repmat(rI,[1 length(t)]),repmat(vI,[1 length(t)]),t,mu);
% %Now ... Convert the propagated Hill results back to an ECI reference
% %frame:
% [rCL,vCL] = Hill2ECI_Vectorized(rTgt,vTgt,rHill,vHill);
% %Compare Results from using CW propagation to the non-linear propagation!
% figure('color',[1 1 1],'Position',[100 200 1000 400]);
% subplot(1,2,1);
% [x,y,z] = sphere(20);
% x = x.*earthRad; y = y.*earthRad; z = z.*earthRad;
% surf(x,y,z); hold on; colormap('bone'); shading flat;
% plot3(rCL(1,:),rCL(2,:),rCL(3,:),'linewidth',3);
% plot3(rChase(1,:),rChase(2,:),rChase(3,:),'--k','linewidth',3);
% axis equal; grid off;
% subplot(1,2,2);
% plot(t./60,-(rChase(1,:)-rCL(1,:)).*1000,'r'); hold on;
% plot(t./60,-(rChase(2,:)-rCL(2,:)).*1000,'k','linewidth',2);
% plot(t./60,-(rChase(3,:)-rCL(3,:)).*1000,'g');
% plot(t./60,-(sqrt(sum(rChase.^2,1))-sqrt(sum(rCL.^2,1))).*1000,'b')
% plot(t./60,-(vChase(1,:)-vCL(1,:)).*1000,'--r'); hold on;
% plot(t./60,-(vChase(2,:)-vCL(2,:)).*1000,'--k');
% plot(t./60,-(vChase(3,:)-vCL(3,:)).*1000,'--g');
% plot(t./60,-(sqrt(sum(vChase.^2,1))-sqrt(sum(vCL.^2,1))).*1000,'--b');
% legend('R_x \epsilon','R_y \epsilon','R_z \epsilon','R \epsilon', ...
%        'V_x \epsilon','V_y \epsilon','V_z \epsilon','V \epsilon','location','SW');
% ylabel('Postion/Velocity Difference (m)/(m/s)');
% xlabel('Popagation Time (min)');
% axis square;
% grid on;
% end

function [rHill,vHill] = CWHPropagator(rHillInit,vHillInit,omega,t)
% Purpose:
% Take initial position and velocity coordinates in the Hill reference frame
% and propagate them using the Clohessy-Wiltshire Hill Linearize equation
% of motion.
%
% Inputs:
%rHillInit                  [3 x 1]                 Hill Position vector
%                                                   (km) / (m)
%
%vHillInit                  [3 x 1]                 Hill Velocity vector of
%                                                   (km/s) / (m/s)
%
%omega                       double                 Orbital Angular Rate
%                                                   of the target
%                                                   (rad/s)
%                                                   Should be close to
%                                                   circular for linear propagation
%                                                   error to be low.
%
%t                          [1 x N]                 Propagation Time in
%                                                   seconds
%
%
%
%
% Outputs:
%rHill                       [3 x N]                Propagated Hill
%                                                   Position vector (km) /
%                                                   (m/s)
%
%vHill                       [3 x N]                Propagated Hill
%                                                   Velocity vector (km/s)
%                                                   / (m/s)
%
%
% References:
%
% Programed by Darin C Koblick 11/30/2012
% Begin Code Sequence

x0 = rHillInit(1,:); 
y0 = rHillInit(2,:); 
z0 = rHillInit(3,:);

x0dot = vHillInit(1,:); 
y0dot = vHillInit(2,:); 
z0dot = vHillInit(3,:);

rHill = [(x0dot./omega).*sin(omega.*t)-(3.*x0+2.*y0dot./omega).*cos(omega.*t)+(4.*x0+2.*y0dot./omega)
    (6.*x0+4.*y0dot./omega).*sin(omega.*t)+2.*(x0dot./omega).*cos(omega.*t)-(6.*omega.*x0+3.*y0dot).*t+(y0-2.*x0dot./omega)];

%z0.*cos(omega.*t)+(z0dot./omega).*sin(omega.*t)];

vHill = [x0dot.*cos(omega.*t)+(3.*omega.*x0+2.*y0dot).*sin(omega.*t)
    (6.*omega.*x0 + 4.*y0dot).*cos(omega.*t) - 2.*x0dot.*sin(omega.*t)-(6.*omega.*x0 + 3.*y0dot)];

%-z0.*omega.*sin(omega.*t)+z0dot.*cos(omega.*t)];
end

function [rHill,vHill] = ECI2Hill_Vectorized(rTgt,vTgt,rChase,vChase)
%% Purpose:
% Convert those position (ECI) and velocity (ECI) into Hill's reference
% frame using both the target and the chaser position/velocity data
%
%% Inputs:
%rTgt                       [3 x N]                 ECI Position vector of
%                                                   reference frame (km)
%
%vTgt                       [3 x N]                 ECI Velocity vector of
%                                                   reference frame (km/s)
%rChase                     [3 x N]
%
%vChase                     [3 x N]
%
%% Outputs:
%rHill                      [3 x N]                 Hill's relative
%                                                   position vector (km)
%
%vHill                      [3 x N]                 Hill's relative
%                                                   velocity vector (km/s)
% References:
% Vallado 2007.
% Programed by Darin C Koblick 11/30/2012
%% Begin Code Sequence
%Declare Local Functions
matrixMultiply = @(x,y)permute(cat(2,sum(permute(x(1,:,:),[3 2 1]).*permute(y,[2 1]),2), ...
                                     sum(permute(x(2,:,:),[3 2 1]).*permute(y,[2 1]),2), ...
                                     sum(permute(x(3,:,:),[3 2 1]).*permute(y,[2 1]),2)),[2 1]);
rTgtMag = sqrt(sum(rTgt.^2,1));
rChaseMag = sqrt(sum(rChase.^2,1));
vTgtMag = sqrt(sum(vTgt.^2,1));
%Determine the RSW transformation matrix from the target frame of reference
RSW = ECI2RSW(rTgt,vTgt);
%Use RSW rotation matrix to convert rChase and vChase to RSW
r_Chase_RSW = matrixMultiply(RSW,rChase);
v_Chase_RSW = matrixMultiply(RSW,vChase);
%Find Rotation angles to go from target to interceptor
phi_chase = asin(r_Chase_RSW(3,:)./rChaseMag);
lambda_chase = atan2(r_Chase_RSW(2,:),r_Chase_RSW(1,:));
CPC = cos(phi_chase);     SPC = sin(phi_chase);
SLC = sin(lambda_chase);  CLC = cos(lambda_chase);
%Find Position component rotations
rHill = cat(1,rChaseMag-rTgtMag, ...
              lambda_chase.*rTgtMag, ...
              phi_chase.*rTgtMag);
%Find the rotation matrix RSW->SEZ of chaser
RSW_SEZ = zeros(3,3,size(rTgtMag,2));
RSW_SEZ(1,1,:) = SPC.*CLC;  RSW_SEZ(1,2,:) = SPC.*SLC;  RSW_SEZ(1,3,:) = -CPC;
RSW_SEZ(2,1,:) = -SLC;  RSW_SEZ(2,2,:) = CLC;
RSW_SEZ(3,1,:) = CPC.*CLC;  RSW_SEZ(3,2,:) = CPC.*SLC;  RSW_SEZ(3,3,:) = SPC;
%Find the velocity component of positions using the angular rates in SEZ frame
v_Chase_SEZ = matrixMultiply(RSW_SEZ,v_Chase_RSW);
vHill = cat(1,v_Chase_SEZ(3,:), ...
              rTgtMag.*(v_Chase_SEZ(2,:)./(rChaseMag.*CPC)-vTgtMag./rTgtMag), ...
              -rTgtMag.*v_Chase_SEZ(1,:)./rChaseMag);
end

function [T,rRSW,vRSW] = ECI2RSW(rECI,vECI)
%% Purpose:
%Convert ECI Coordinates to RSW Coordinates, also, return the
%transformation matrix T in which to take a given set of coordinates in ECI
%and convert them using the same RSW reference frame.
%
%% Inputs:
% rECI              [3 x N]                     ECI position Coordinates in
%                                               km
%
% vECI              [3 x N]                     ECI velocity Coordinates in
%                                               km/s
%
%% Outputs:
% T                 [3 x 3 x N]                 Transformation matrix
%                                               necessary to go from
%                                               rECI -> rRSW
%
% rRSW              [3 x N]                     RSW Position Coordinates
%                                               km
%
% vRSW              [3 x N]                     RSW Velocity Coordinates
%                                               km/s
%
%% References:
% Vallado pg. 173
% Vallado rv2rsw.m code dated 06/09/2002
%
%Programmed by: Darin C Koblick                 11/29/2012
%% Begin Code Sequence
if nargin == 0
    rECI =  repmat([6968.1363,1,2]',[1 10]);
    vECI =  repmat([3,7.90536615282099,4]',[1 10]);
    [T,rRSW,vRSW] = ECI2RSW(rECI,vECI);
    return;
end
%Declared Internal function set:
unitv = @(x)bsxfun(@rdivide,x,sqrt(sum(x.^2,1)));
matrixMultiply = @(x,y)permute(cat(2,sum(permute(x(1,:,:),[3 2 1]).*permute(y,[2 1]),2), ...
                                     sum(permute(x(2,:,:),[3 2 1]).*permute(y,[2 1]),2), ...
                                     sum(permute(x(3,:,:),[3 2 1]).*permute(y,[2 1]),2)),[2 1]);
%Find the Radial component of the RIC position vector
rvec = unitv(rECI);
%Find the cross-track component of the RIC position vector
wvec = unitv(cross(rECI,vECI));
%Find the along-track component of the RIC position vector
svec = unitv(cross(wvec,rvec));
%Create the transformation matrix from ECI to RSW
T = NaN(3,3,size(rECI,2));
T(1,1,:) = rvec(1,:); T(1,2,:) = rvec(2,:); T(1,3,:) = rvec(3,:);
T(2,1,:) = svec(1,:); T(2,2,:) = svec(2,:); T(2,3,:) = svec(3,:);
T(3,1,:) = wvec(1,:); T(3,2,:) = wvec(2,:); T(3,3,:) = wvec(3,:);
%Find the position and velocity vectors in the RSW reference frame!
rRSW = matrixMultiply(T,rECI);
vRSW = matrixMultiply(T,vECI);
end


function [rInt,vInt] = Hill2ECI_Vectorized(rTgt,vTgt,rHill,vHill)
%% Purpose:
% Convert those position (rHill) and velocity (vHill) values back into an
% ECI coordinate frame of reference using the reference satellite
% (rTgt,vTgt) position and velocity data.
%
%% Inputs:
%rTgt                       [3 x N]                 ECI Position vector of
%                                                   reference frame (km)
%
%vTgt                       [3 x N]                 ECI Velocity vector of
%                                                   reference frame (km/s)
%
%rHill                      [3 x N]                 Hill's relative
%                                                   position vector (km)
%
%vHill                      [3 x N]                 Hill's relative
%                                                   velocity vector (km/s)
%
%
%
%% Outputs:
%rInt                       [3 x N]
%
%vInt                       [3 x N]
%
%
% References:
% Vallado 2007.
% Programed by Darin C Koblick 11/30/2012
%% Begin Code Sequence
%Declare Local Functions
rTgtMag = sqrt(sum(rTgt.^2,1));
vTgtMag = sqrt(sum(vTgt.^2,1));
matrixMultiply = @(x,y)permute(cat(2,sum(permute(x(1,:,:),[3 2 1]).*permute(y,[2 1]),2), ...
                                     sum(permute(x(2,:,:),[3 2 1]).*permute(y,[2 1]),2), ...
                                     sum(permute(x(3,:,:),[3 2 1]).*permute(y,[2 1]),2)),[2 1]);
%Find the RSW matrix from the target ECI positions
RSW = ECI2RSW(rTgt,vTgt); rIntMag = rTgtMag + rHill(1,:);
%Compute rotation angles to go from tgt to interceptor
lambda_int = rHill(2,:)./rTgtMag; phi_int = sin(rHill(3,:)./rTgtMag);
CLI = cos(lambda_int); SLI = sin(lambda_int); CPI = cos(phi_int); SPI = sin(phi_int);
%find rotation matrix to go from rsw to SEZ of inerceptor
RSW_SEZ = zeros(3,3,size(rTgt,2));
RSW_SEZ(1,1,:) = SPI.*CLI;  RSW_SEZ(1,2,:) = SPI.*SLI; RSW_SEZ(1,3,:) = -CPI;
RSW_SEZ(2,1,:) = -SLI;      RSW_SEZ(2,2,:) = CLI;      RSW_SEZ(3,1,:) = CPI.*CLI;
                            RSW_SEZ(3,2,:) = CPI.*SLI; RSW_SEZ(3,3,:) = SPI;
%Find velocity component positions by using angular rates in SEZ frame
vIntSEZ = cat(1,-rIntMag.*vHill(3,:)./rTgtMag, ...
                 rIntMag.*(vHill(2,:)./rTgtMag + vTgtMag./rTgtMag).*CPI, ...
                 vHill(1,:));
vInt = matrixMultiply(permute(RSW,[2 1 3]), ...
       matrixMultiply(permute(RSW_SEZ,[2 1 3]), ...
       vIntSEZ));
%Find the position components
rIntRSW = bsxfun(@times,rIntMag,cat(1,CPI.*CLI, ...
                                      CPI.*SLI, ...
                                      SPI));
rInt = matrixMultiply(permute(RSW,[2 1 3]),rIntRSW);      
end


function [r,v] = keplerUniversal(r0,v0,t,mu)
%Purpose:
%Most effecient way to propagate any type of two body orbit using kepler's
%equations.
%-------------------------------------------------------------------------%
%                                                                         %
% Inputs:                                                                 %
%--------                                                                  
%r_ECI                  [3 x N]                         Position Vector in
%                                                       ECI coordinate
%                                                       frame of reference
%
%v_ECI                  [3 x N]                         Velocity vector in
%                                                       ECI coordinate
%                                                       frame of reference
%
%t                      [1 x N]                         time vector in
%                                                       seconds
%
%mu                     double                          Gravitational Constant
%                                                       Defaults to Earth if
%                                                       not specified
% Outputs:
%---------                                                                %
%r_ECI                  [3 x N]                         Final position
%                                                       vector in ECI
%
%v_ECI                  [3 x N]                         Final velocity
%                                                       vector in ECI
%--------------------------------------------------------------------------
% Programmed by Darin Koblick 03-04-2012                                  %
%-------------------------------------------------------------------------- 
if ~exist('mu','var'); mu = 398600.4418; end
tol = 1e-9;
v0Mag = sqrt(sum(v0.^2,1));  r0Mag = sqrt(sum(r0.^2,1));
alpha = -(v0Mag.^2)./mu + 2./r0Mag; 
%% Compute initial guess (X0) for Newton's Method
X0 = NaN(size(t));
%Check if there are any Eliptic/Circular orbits
idx = alpha > 0.000001;
if any(idx)
    X0(idx) = sqrt(mu).*t(idx).*alpha(idx); 
end
%Check if there are any Parabolic orbits
idx = abs(alpha) < 0.000001;
if any(idx)
   h = cross(r0(:,idx),v0(:,idx)); hMag = sqrt(sum(h.^2,1));
   p = (hMag.^2)./mu; s = acot(3.*sqrt(mu./(p.^3)).*t(idx))./2;
   w = atan(tan(s).^(1/3)); X0(idx) = sqrt(p).*2.*cot(2.*w);
end
%Check if there are any Hyperbolic orbits
idx = alpha < -0.000001;
if any(idx)
   a = 1./alpha(idx);
   X0(idx) = sign(t(idx)).*sqrt(-a).*...
       log(-2.*mu.*alpha(idx).*t(idx)./ ...
       (dot(r0(:,idx),v0(:,idx))+sign(t(idx)).*sqrt(-mu.*a).*...
       (1-r0Mag(idx).*alpha(idx))));
end
%% Newton's Method to converge on solution
% Declare Constants that do not need to be computed within the while loop
err = Inf;
dr0v0Smu = dot(r0,v0)./sqrt(mu);
Smut = sqrt(mu).*t;
while any(abs(err) > tol)
    X02 = X0.^2;
    X03 = X02.*X0;
    psi = X02.*alpha;
    [c2,c3] = c2c3(psi);
    X0tOmPsiC3 = X0.*(1-psi.*c3);
    X02tC2 = X02.*c2;
    r = X02tC2 + dr0v0Smu.*X0tOmPsiC3 + r0Mag.*(1-psi.*c2);
    Xn = X0 + (Smut-X03.*c3-dr0v0Smu.*X02tC2-r0Mag.*X0tOmPsiC3)./r;
    err = Xn-X0; X0 = Xn;
end
f = 1 - (Xn.^2).*c2./r0Mag; g = t - (Xn.^3).*c3./sqrt(mu);
gdot = 1 - c2.*(Xn.^2)./r; fdot = Xn.*(psi.*c3-1).*sqrt(mu)./(r.*r0Mag);
r = bsxfun(@times,f,r0) + bsxfun(@times,g,v0);
v = bsxfun(@times,fdot,r0) + bsxfun(@times,gdot,v0);
%% Ensure Solution Integrity
%idx = round((f.*gdot - fdot.*g)./tol).*tol ~= 1; r(:,idx) = NaN; v(:,idx) = NaN;
end
function [c2,c3] = c2c3(psi)
%Vallado pg. 71 Algorithm 1
c2 = NaN(size(psi));
c3 = NaN(size(psi));
idx = psi > 1e-6;
if any(idx)
    c2(idx) = (1-cos(sqrt(psi(idx))))./psi(idx);
    c3(idx) = (sqrt(psi(idx))-sin(sqrt(psi(idx))))./sqrt(psi(idx).^3);
end
idx = psi < -1e-6;
if any(idx)
    c2(idx) = (1 - cosh(sqrt(-psi(idx))))./psi(idx);
    c3(idx) = (sinh(sqrt(-psi(idx)))-sqrt(-psi(idx)))./sqrt(-psi(idx).^3);
end
idx = abs(psi) <= 1e-6;
if any(idx)
    c2(idx) = 0.5;
    c3(idx) = 1/6;
end
end