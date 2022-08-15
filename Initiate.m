clear all
close all
clc

% Author: Fedor van der Laan
% Year: 2022
% Contact: fedorvanderlaan at gmail dot com
% University: TU Delft
% Code to simulate the behavior of the fault tolerant flashback control
% system to support the master thesis to obtain the double degree in
% Mechanical engineering and Systems & Control

%% Simulink model of the fault tolerant flashback control system
% System variables
fs = 50;                                % Sampling frequency [Hz]
ts = 1/fs;                              % Sampling time [s]
tmax = 400;                             % Simulation length [s]
t = 0:ts:tmax;                          % Time frame [s]
dTulim = 3.8;                           % Flashback detection limit [C]
Dtmax = 2;                              % Max time to recover from flashback [s]
eta_switch = 0.05;                      % Stop back from countermeasure to prevention controller if within limit [-]

% Get experimental data to initiate Temperature and Reynolds values from
% experiments
load("data.mat")

%% System models

% State space model non-flashback model
Anom = 0.9989;
Bnom = -1.4118e-05;   
Cnom = 1;

% State space model flashback model
Afb = [1.9325, 1;
       -0.9328, 0];
Bfb = [16.4988; 
       -16.4996];   
Cfb = [1, 0];   
   
% State space model counteraction   
Acure = Afb;
Bcure = 2/7*Bfb;
Cnf = 1;

% State space model no flame condition
Anf = 0.9855;
Bnf = -1.5360/4;
Ccure = Cfb;

%% Reference temperature
Tref = 55*ones(size(t));
Tref(200/ts:400/ts) = 62; 

%% Succeed or fail CM

% Decide from which time point the counteraction is no longer succesfully
% counteracted
CMsucces = ones(size(t));
CMsucces(300/ts:end) = 0;

%% Pole
% Pole placement controller
F = [107.404393816111,9.22073025292975];

%% Run simulink model 
SimOptions.ReturnWorkspaceOutputs = 'on';
out = sim('FlshbcontrolHybridefinal.slx',400, simset(SimOptions));

%% Plots
% Active controllers
figure
hold on
plot(t,out.Activecontroller,'LineWidth',1.5)
ylabel('\delta')
legend('\delta_{cm}','\delta_{switch}')
xlabel('t[s]')
xlim([230,350])
set(gca,'FontSize',12);
hold off

% Simulation result
figure
hold on
plot(t(230*fs:400*fs),out.Simdata(230*fs:400*fs,1:2),'LineWidth',1)
plot(t(327*fs:400*fs),out.Simdata(327*fs:400*fs,2),'c','LineWidth',1)
plot(t(1:length(out.Simdata)),out.Simdata(:,3),'k-.','LineWidth',1)
legend('T_{ref}','T_{flame}','T_{noflame}','T_{flsh limit}')
set(gca,'FontSize',12);
ylabel('T[C]')
xlabel('t[s]')
ylim([40,300])
xlim([230,350])
hold off

% Variable switch
figure
hold on
plot(t,out.SimdataFB,'LineWidth',1.5)
plot(t,out.SimdataCM,'LineWidth',1.5)
legend('\delta_{fb}','\delta_{cm}')
ylabel('\delta')
xlabel('t[s]')
xlim([234,236])
set(gca,'FontSize',12);
hold off

% Peak detection
figure
hold on
plot(t,out.SimdataDT,'LineWidth',1)
plot(t,out.SimdataLimit,'LineWidth',1)
plot(t([11752,11843]),out.SimdataDT([11752,11843]),'c*')
ylabel('\Delta T[C]')
xlabel('t[s]')
h1 = legend('$$\dot{T}$$','$$\dot{T}_{ulimit}$$','peak detected');
set(h1,'Interpreter','latex');
xlim([230,250])
set(gca,'FontSize',12);
hold off

% Simulation flashback
figure
hold on
plot(t(230*fs:280*fs),out.Simdata(230*fs:280*fs,1:2),'LineWidth',1)
plot(t(1:length(out.Simdata)),out.Simdata(:,3),'k-.','LineWidth',1)
legend('T_{ref}','T_{sim}','T_{flsh limit}')
set(gca,'FontSize',12);
ylabel('T[C]')
xlabel('t[s]')
ylim([40,300])
xlim([230,260])
hold off

% Controller output
figure
hold on
plot(t,out.SimdataRe,'LineWidth',1)
legend('Controller output')
xlabel('t[s]')
ylabel('Re[-]')
ylim([2000, 6500])
hold off

% Simulation disturbance rejection
figure
hold on
plot(t(1:180*fs),out.Simdata(1:180*fs,1:2),'LineWidth',1)
plot(t(1:180*fs),55+out.distout(1:180*fs,1))
legend('T_{ref}','T','T_{with dist}')
ylabel('T[C]')
xlabel('t[s]')
ylim([45,65])
xlim([0,180])
hold off