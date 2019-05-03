function [sys,x0,str,ts] = quadrotor_backstepping_controllerfinxy(t,x,u,flag)
%quadrotorsfunc An example MATLAB file S-function for defining a continuous system.  
%   Example MATLAB file S-function implementing continuous equations: 
%      x' = f(x,u)
%      y  = h(x)
%   See sfuntmpl.m for a general S-function template.
%   See also SFUNTMPL.
%   Copyright 1990-2009 The MathWorks, Inc.
switch flag,
  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts] = mdlInitializeSizes;
  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);
  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);
  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
% end quadrotorsfunc

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 11;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [1; 0; 1; 0; 1; 0; 1; 0; 1; 0; 1; 0];
%x0  = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
str = [];
ts  = [0 0];
% end mdlInitializeSizes

%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
function sys=mdlDerivatives(t,x,u)
g   =9.81;     l  = 0.45;    %wr  =  u(5);
wr  =1;        Jr = 6e-3;      m = 2;
Ixx =0.018125; Iyy= 0.018125; Izz= 0.035; %wr = 2577;

%%
a1 = (Iyy-Izz)/Ixx; a2 = Jr/Ixx; a3 = (Izz-Ixx)/Iyy; a4 = Jr/Iyy;
a5 = (Ixx-Iyy)/Izz; b1 = l/Ixx; b2 = l/Iyy; b3 = l/Izz;

xdot    = [x(2);
           x(4)*x(6)*a1 - x(4)*wr*a2 + b1*u(2);
           x(4);
           x(2)*x(6)*a3 + x(2)*wr*a4 + b2*u(3);
           x(6);
           x(2)*x(4)*a5 + b3*u(4)
           x(8);
           g - (u(1)/m)*(cosd(x(1))*cosd(x(3)));
           x(10);
           - (u(1)/m)*(sind(x(1))*sind(x(5)) + cosd(x(1))*sind(x(3))*cosd(x(5)));
           x(12);
           - (u(1)/m)*(sind(x(1))*cosd(x(5)) - cosd(x(1))*sind(x(3))*sind(x(5)));];
sys = xdot;
% end mdlDerivatives

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
function sys=mdlOutputs(t,x,u)
g    = 9.81;     l    =  0.45;    %wr   =  u(5);
wr   = 1;        Jr   =  6e-3;     m    =  2;
Ixx  = 0.018125; Iyy  =  0.018125; Izz  =  0.035000;
a1 = (Iyy-Izz)/Ixx; a2 = Jr/Ixx; a3 = (Izz-Ixx)/Iyy; a4 = Jr/Iyy;
a5 = (Ixx-Iyy)/Izz; b1 = l/Ixx; b2 = l/Iyy; b3 = l/Izz;
%wr = 2577; %m = 1.8;

% %DESIRED SET VALUES !
% % x5d   = 5;  %ksi desired
% % x7d   = 10; %z - desired
% % x9d   = 20; %x - desired
% % x11d  = 15; %y - desired
% 
% x5d   = 5;  %ksi desired
% x7d   = 5; %z - desired
% x9d   = 5; %x - desired
% x11d  = 5; %y - desired
% 
% x1ddot    = 0;
% x3ddot    = 0;
% x5ddot    = 0;
% x7ddot    = 0;
% x1d2dot   = 0;%outer
% x3d2dot   = 0;
% x5d2dot   = 0;
% x7d2dot   = 0;
% x10d2dot  = 0;%inner
% x12d2dot  = 0;%
% 
% % c1  =  5.52;
% % c2  =  3.40;
% % c3  =  3.00;
% % c4  =  2.50;
% 
% c1  =  u(9);
% c2  =  u(10);
% c3  =  u(11);
% c4  =  u(12);
% 
% c5  =  3.07;
% c6  =  4.71;
% c7  =  6.11;
% c8  =  7.96;
% 
% tt = u(5);
% 
% z5  =  x5d - x(5);
% z6  =  x(6)- x5ddot - c5*z5;
% z7  =  x7d - x(7);
% z8  =  x(8)- x7ddot - c7*z7;
% 
% 
% %--------------------------------------------------------------------------
% u1   =  (m/(cosd(x(1))*cosd(x(3))))*(-z7 + g - x7d2dot - c7*x7ddot + c7*x(8) + c8*z8);      % z
% 
% % kpy    = 13; % kdy    = 13;  % kpx    = 13; % kdx    = 13;
%   kpy    = 3.9;  kdy    = 8.3;   kpx    = 3.9;  kdx    = 7.3;
% 
% kpx = u(6);
% kdx = u(7);
% kpy = u(8);
% kdy = u(9);
% 
% phid   =  kpy*(x(11) - x11d) + kdy*(x(12) - x12d2dot);
% thetad =  kpx*(x(9) - x9d)   + kdx*(x(10) - x10d2dot);
% x1d    =  cosd(x(5))*phid - sind(x(5))*thetad;
% x3d    =  sind(x(5))*phid + cosd(x(5))*thetad;

%--------------------------------------------------------------------------

x5d = 5;     %ksi
x7d = 5;     %z
% x9d = u(5);  %x
% x11d= u(6);  %y
x9d = 5;  %x
x11d= 5;  %y

x1ddot = 0;x3ddot  =0; x5ddot =0;x7ddot  =0;x1d2dot =0;
x3d2dot= 0;x5d2dot =0;x7d2dot =0;x10d2dot=0;x12d2dot=0;

% c1 = 5.52;c2 = 3.40;c3 = 3.00;c4 = 2.50;
% c5 = 3.07;c6 = 4.71;c7 = 6.11;c8 = 7.96;
c1  =  11.52;
c2  =  8.40;
c3  =  8.00;
c4  =  7.50;
c5  =  13.6263;
c6  =  13.5392;
c7  = 2.54;
c8  = 5.49;
%c1  =  u(10);c2  =  u(11);c3  =  u(12);c4  =  u(13);

% kpy = 3.9; kdy = 8.3; 
% kpx = 3.9; kdx = 7.3;

kpy = 4.9; kdy = 8.3; 
kpx = 4.9; kdx = 7.3;

% kpx = u(6);
% kdx = u(7);
% kpy = u(8);
% kdy = u(9);

z5 = x5d - x(5); 
z6 = x(6)- x5ddot - c5*z5;
z7 = x7d - x(7); 
z8 = x(8)- x7ddot - c7*z7;

u1 = (m/(cosd(x(1))*cosd(x(3))))*(-z7 + g - x7d2dot - c7*x7ddot + c7*x(8) + c8*z8);      % z
  phid   =  kpy*(x(11) - x11d) + kdy*(x(12) - x12d2dot);
  thetad =  kpx*(x(9) - x9d)  + kdx*(x(10) - x10d2dot);
%     phid   =  -(kpy*(x11d - x(11)) + kdy*(x12d2dot - x(12)));
%     thetad =  -(kpx*(x9d - x(9))   + kdx*(x10d2dot - x(10)));
x1d = (cosd(x(5))*phid - sind(x(5))*thetad);
x3d = (sind(x(5))*phid + cosd(x(5))*thetad);

z1    =   x1d - x(1);
z2    =   x(2)- x1ddot - c1*z1;
z3    =   x3d - x(3);
z4    =   x(4)- x3ddot - c3*z3;

u2    =  (1/b1)*(-c2*z2 + z1 - x(4)*x(6)*a1 + x(4)*wr*a2 + x1d2dot + c1*x1ddot - c1*x(2)); % phi
u3    =  (1/b2)*(-c4*z4 + z3 - x(2)*x(6)*a3 - x(2)*wr*a4 + x3d2dot + c3*x3ddot - c3*x(4)); % theta
u4    =  (1/b3)*(-c6*z6 + z5 - x(2)*x(4)*a5 + x5d2dot + c5*x5ddot - c5*x(6));              % ksi

%wwr = 257;

sys = [ x(1); x(3); x(5); x(7); x(9); x(11); u1; u2; u3; u4; 1;];
% end mdlOutputs
