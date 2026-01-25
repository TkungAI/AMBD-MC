clear;clc;

%% Load Structures
load('struct_FOC_Crtl.mat');

%% Model parameters
Ts = 0.0001;
Tctr = 0.0000625;
Ts_PIL = 0.0000125;
Fpwm = 16000;
Ts_simscape = 1/40000000;

%% Motor parameters
motor.DC = 12;                      % [V]
motor.Rs = 0.56;                    % [ohm]
motor.Ld = 0.000375;                % [H]
motor.Lq = 0.000435;                % [H]
motor.L0 = (motor.Ld + motor.Ld)/2; % [H]
motor.Prs = 2;                      % [-]
motor.Flux = 0.0039052261;          % [Wb]
motor.J = 0.12e-4;                  % [Kg.m^2]
motor.B = 0.0005;                   % [N.m.s/rad]
motor.Tf = 0;                       % [N.m]
motor.Kt = 1.5*motor.Flux*motor.Prs;                      % Torque constant [N.m/A]
motor.Ke = sqrt(3)*1000*motor.Prs*2*pi/60*motor.Flux;     % Back-emf [V/krpm]

%% Current Controller
% D axis PI design
innerPI.iD.f0 = 200;
innerPI.iD.w0 = 2*pi*innerPI.iD.f0;
innerPI.iD.ksi = 1;

innerPI.iD.Continuous.Kp = 2*innerPI.iD.ksi*innerPI.iD.w0*motor.Ld - motor.Rs;
innerPI.iD.Continuous.Ki = innerPI.iD.w0^2*motor.Ld;
innerPI.iD.Continuous.Kzc = innerPI.iD.Continuous.Kp/innerPI.iD.Continuous.Ki;

innerPI.iD.Discrete.Kp = 10;
innerPI.iD.Discrete.Ki = 480;
innerPI.iD.Discrete.KiTctr = innerPI.iD.Discrete.Ki*Tctr;

% Q axis PI design
innerPI.iQ.f0 = 200;
innerPI.iQ.w0 = 2*pi*innerPI.iQ.f0;
innerPI.iQ.ksi = 1;

innerPI.iQ.Continuous.Kp = 2*innerPI.iQ.ksi*innerPI.iQ.w0*motor.Lq - motor.Rs;
innerPI.iQ.Continuous.Ki = innerPI.iQ.w0^2*motor.Lq;
innerPI.iQ.Continuous.Kzc = innerPI.iQ.Continuous.Kp/innerPI.iQ.Continuous.Ki;

innerPI.iQ.Discrete.Kp = 10;
innerPI.iQ.Discrete.Ki = 640;
innerPI.iQ.Discrete.KiTctr = innerPI.iQ.Discrete.Ki*Tctr;

%% Speed Controller
outerPI.Spd.f0 = 4;
outerPI.Spd.w0 = 2*pi*outerPI.Spd.f0;
outerPI.Spd.ksi = 1;

outerPI.Spd.Continuous.Kp = (2*outerPI.Spd.ksi*outerPI.Spd.w0*motor.J - motor.B)/motor.Kt;
outerPI.Spd.Continuous.Ki = outerPI.Spd.w0^2*motor.J/motor.Kt;
outerPI.Spd.Continuous.Kzc = outerPI.Spd.Continuous.Kp/outerPI.Spd.Continuous.Ki;

outerPI.Spd.Discrete.Kp = 0.07;
outerPI.Spd.Discrete.Ki = 0.32;
outerPI.Spd.Discrete.KiTctr = outerPI.Spd.Discrete.Ki*Tctr;

