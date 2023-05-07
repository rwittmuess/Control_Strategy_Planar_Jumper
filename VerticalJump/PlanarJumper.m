function PlanarJumper
%% Legged Robots - Final Project
%{
    ToDo:
    * make event function dependend on l'' < -g for lift-off
    * plot desired path and actual path to make sure robot follows desired path
    * 
    * adapt landing to be smooth
    * calculate K_i's for desired height/speed/time,...

%}


%% For a fresh start:
clc; clear; close all;
% dynamics;


%% activate Profiler
% profile on
% Run code to profile


%% Settings for increasing computational speed
reltol_start    = 1e-2;
abstol_start    = 1e-3;
tspan_start     = 0:0.01:3;   % tspan_start = [0 1];

reltol_flight   = 1e-2;
abstol_flight   = 1e-3;
% tspan_flight    = [0:0.01:1];   % tspan_start = [0 1];

reltol_landing = 1e-2;
abstol_landing = 1e-3;

%% GROUND PHASE
%s0 = [0;-pi/4;pi/2;0;0;...
      % 0;0;0;0;0];

s0 = [0;-pi/4;pi/2;0;0;...
      0;0;0;0;0];

Opt = odeset('Events', @robotics_event,'RelTol',reltol_start,'AbsTol',abstol_start);
[t,s] = ode45(@(t,s) robot_dynamics(t,s), tspan_start, s0, Opt);

tData = t(1:end-1);
sData = s(1:end-1,:);
tData_GroundPhase = t;
sData_GroundPhase = s;

% qData = sData(:,1:5);
% animateRobot(tData,qData)


%% FLIGHT PHASE
% s0 = [0.643551279512161, -1.38116878810655, 0.737613340437614, -4.36564827749447e-15, 2.25977590679604e-14, -1.04637683867421, 2.32542793026359, -1.28181713291239, -6.02979306700862e-15, 3.01614185027691e-14];
s0 = sData(end,:);

LOv = [l_gen(sData(end,1:5)');dl_gen(sData(end,:)')];

Opt = odeset('Events', @robotics_flight_event,'RelTol',reltol_flight,'AbsTol',abstol_flight);
[t,s] = ode45(@(t,s) robot_dynamics_flight(t,s,LOv), tData(end):0.01:tData(end)+3, s0, Opt); %[0:0.005:0.15]

% tData = [tData;tData(end)+t(2:end)];
tData = [tData;t(2:end)];
sData = [sData;s(2:end,:)];
tData_FlightPhase = t;
sData_FlightPhase = s;


%% Get new trajectory for LANDING PHASE
% % https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=4758204
% % https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=4755928
% 
% % get position where velocity is 0
% dzCOM_array = dzCOM_gen(sData_GroundPhase(:,:)');
% dzCOM_array = dzCOM_array(2:end); % to avoid the first entry as the speed will be 0 there.
% 
% % Find the index where the values go from negative to positive
% index = find(diff(sign(dzCOM_array)) > 0, 1);
% disp(index);
% disp(dzCOM_array(index))
% disp(dzCOM_array(index+1))

%% LANDING PHASE
s0 = sData(end,:);
s0(9:10) = [0.0,0.0]; % to satisfy assumptions of the paper

% -> = [0.7122   -1.6219    0.9931   -0.0000   -0.0052    0.6455   -2.2303  2.2222   -0.5956   -0.0477]; we get this
%--> = [0.7122   -1.6219    0.9931   -0.0000   -0.0052    0.6455   -2.2303  2.2222   -0.0000   -0.0000]; we modify it to this
% s0 = [0.6437   -1.3817    0.7380   -0.0000   -0.0000   -1.2026    2.5749 -1.4090   -0.0000   -0.0000]; % s(end) from Ground_Phase

t_LI = tData(end);

Opt = odeset('Events', @(t,s)robotics_landing_event(t,s,t_LI),'RelTol',reltol_start,'AbsTol',abstol_start);
[t,s] = ode45(@(t,s) robot_dynamics_landing(t,s,t_LI), 0:0.01:2, s0, Opt);

t_corrected = t(2:end)+tData(end);
tData = [tData;t_corrected];
sData = [sData;s(2:end,:)];

tData_BouncePhase = t_corrected;
sData_BouncePhase = s;


%% JUST TO TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% qData = sData_BouncePhase(:,1:5);
% animateRobot(tData_BouncePhase,qData)

%% prepare next jump
% s0 = sData(end,:);
% 
% Opt = odeset('Events', @robotics_event,'RelTol',reltol_start,'AbsTol',abstol_start);
% [t,s] = ode45(@(t,s) robot_dynamics(t,s), tData(end):0.01:tData(end)+3, s0, Opt);
% 
% tData = [tData;t(2:end)];
% sData = [sData;s(2:end,:)];


%% deactivate Profiler
% Path where you want to store the HTML profiler results
% html_folder = './Profiler';
% profsave(profile('info'), html_folder)


%%
% t = 0;
% q = [0,-pi/4,pi/2,0,0];

qData = sData(:,1:5);
animateRobot(tData,qData)


%%
% lref_LO = lref_LO_gen(tData);
% l = l_gen(qData')';
% % t1s_val = [tData';ones(length(tData)*1)];
% % lref_F = lref_F_gen(t1s_val,sData);
% % lref_F = [zeros(1,101),lref_F]';
% % zFref = zFref_gen(t1s_val);
% 
% %%
% 
% % ds = robot_dynamics(tData,sData');
% 
% figure(1)
% plot(tData,l);
% hold on;
% grid on;
% plot(tData,lref_LO);
% legend(["l_{real}", "lref_{LO}"]);
% 
% dlref_LO = dlref_LO_gen(tData);
% dl = dl_gen(sData')';
% 
% figure(2)
% plot(tData,dl);
% hold on;
% grid on;
% plot(tData,dlref_LO);
% legend(["dl_{real}", "dlref_{LO}"])
% % 
% % ds2 = [sData, ds(6:end,:)'];
% % 
% % ddl = ddl_gen(ds2')';
% % 
% % ddl = diff(dl)/(tData(2)-tData(1));
% 
% %%
% tF = tData(101:end);
% t1s_val = [tF';ones(1,length(tF))*1];
% %%
% zFref = zFref_gen(t1s_val);
% figure(3)
% hold on;
% grid on;
% plot(tF,zFref);
% plot(tF,sData(101:end,4));
% legend(["zF_{ref}","zF_{real}"]);
% 
% %%
% % lref_F = lref_F_gen([tF';ones(1,length(tF))*1;ones(1,length(tF))*LOv(1);ones(1,length(tF))*LOv(2)]);
% zCOM_kin = zCOM_kin_gen([tF';ones(1,length(tF))*1;ones(1,length(tF))*LOv(1);ones(1,length(tF))*LOv(2)]);
% zFref = zFref_gen([tF';ones(1,length(tF))*1]);
% pCOM = pCOM_gen(sData(101:end,1:5)');
% zCOM = pCOM(2,:);
% 
% lref_F = zCOM - zFref;
% 
% figure(4)
% hold on;
% grid on;
% plot(tF,lref_F);
% plot(tF,zCOM_kin);
% plot(tF,zFref);
% % plot(tF,sData(102:end,4));
% plot(tF,zCOM);
% plot(tF,l(101:end));
% 
% legend(["lref_F", "zCOM_{kin}", "zF_{ref}", "zCOM_{real}","l_{real}"]);
% % plot(tF,sData(102:end,4));
% 
% 
% dzCOM_kin = dzCOM_kin_gen([tF';ones(1,length(tF))*1;ones(1,length(tF))*LOv(1);ones(1,length(tF))*LOv(2)]);
% dzCOM = dzCOM_gen(sData(101:end,:)');
% 
% 
% 
% figure(5)
% hold on;
% grid on;
% plot(tF,dzCOM_kin);
% plot(tF,dzCOM);
% legend(["dzCOM_{kin}", "dzCOM_{real}"])
% 
% tTest = 1:0.01:1.5;
% zFref = zFref_gen([tTest;ones(1,length(tTest))*1]);
% 
% figure(6)
% hold on;
% grid on;
% plot(tTest,zFref);

end