% runSimLoader - Script to run the wheel_loader_dynamic model
%
% Simulates wheel_loader_dynamic.slx for modeling the loading process. The
% basic information about the wheel loader is as following:
% It is a Volvo L220G wheel loader (2012 version) with automatic power
% shift system (Volvo HTL 306: 4th forward/reverse gear ratios). The 12.81L non-engine
% of the employed loader is D13H-E (Tier 4i) or D13H-F (Stage IIIB) whose maximun power is
% 274 / 373 (kW / hp) @ 1300-1400rpm. All the parameters and control inputs
% used in this simulation come from the publishments by VOLVO CE or
% Vaheed Nezhadali and Lars Eriksson's pulished papers.

%    Copyright 2016, Beichuan Hong
%

clear;clc;

%% parameters of the engine and powertrain system
% Torque calculation
eta_ig = 0.3623;  %combustion charmber efficiency
q_hv = 42.9e6; %heat value unit: J/kg
n_cyl = 6; %cylinder number
Vd = 13e-3; %engine displacement value unit:m^3
% the definition of tunning parameters
% engine friction coeffcient
c_fri_1 = 0.7196;
c_fri_2 = -.01414;
c_fri_3 = 0.3590;
% intake manifold pressure parameters
c_p1 = -0.328;
c_p2 = -121.519;
c_p3 = 0.057;
c_p4 = 97179.699;
c_p_m = [c_p1,c_p2,c_p3,c_p4];
% time consumption parameters for turbo-delay
c_t1 = 38.5857;
c_t2 = -0.6869;
% intake airflow calculation
eta_vol_eng = 0.9; % Volumetric efficiency of diesel engine
Ra = 287; % air constant: unit: J/kg*K
T_amb = 21 + 273.15; % the ambient temperature
% smoke limitations
lambda_minimum = 1.2; % the min of the air-fuel ratio;
% engine rotation
I_ice = 3; % engine inertia unit:kg*m^2;

% powertrain
gear_ratio =[-60 0 60] ;
eta_gb = 0.9; % efficiency of gearbox
r_w = 0.7; % wheel radius; unit: m

% vehicle system
% M_load = 23000; %the mass of loading; unit:kg
% M_boom = 5500; %the weight of the boom structure; unit:kg
J_w = 1000; %the inertia of one wheel; unit:kg*m^2
M_buc = 10000; %the mass of bucket; unit:kg
M_veh = 32000; %the mass of wheel loader; unit:kg
M_wheels = 4 .* J_w ./ r_w.^2;
c_r = 0.03; %rolling resistance of wheel loader

% steering system
% control inputs limitations
u_str_max = 1 .* 180 ./ pi;
u_str_min = -1 .* 180 ./ pi;
L = 3.7; %the length between the front and rear axles unit:m
c_p = 3e4; % steering power parameter

% Lifting system
% Hydrulic pumps
r_pis = 0.19./2; % radius of lift piston
r_rod = 0.09./2; % radius of lift rod
A_cylinder = pi * (r_pis^2 - r_rod^2);
D_pump = 220/1900; % Hydraulic pump displacement
eta_volumetric = 0.98; % Volumetric efficiency of the lift pump
eta_lift = 0.9; % efficiency of lift pump
n_cyl_hydra = 2; % Cylider numbers of the hydratic system
% Geometry of lifting system
r = 2.9; % the boom length;
r1 = 1.7; % the boom length from body joint (O) to connection
xc = 0.19; % dimension horizontal axis
yc = -0.3; % dimension vertical axis
% Loading paremeters
M_load = 10000; %the mass of loading; unit:kg
M_boom = 5000; %the weight of the boom structure; unit:kg
% Generate the geometry efficiency
theta = -40:0.01:80;
[k_theta,theta_1] = K_theta(theta,r,r1,xc,yc);

%% simulate the model of loader
sim('wheel_loader_dynamic',8)
% simulation outputs
time = A_simout_steering_dynamic.time;
X_position  = A_simout_steering_dynamic.signals.values(:,1);
Y_position  = A_simout_steering_dynamic.signals.values(:,2);
% powertrain
engine_speed = A_simout_powertrain.signals.values(:,2);
Te = A_simout_powertrain.signals.values(:,1);
Pm = A_simout_powertrain.signals.values(:,4);
P_im = A_simout_powertrain.signals.values(:,3);
V = A_simout_powertrain.signals.values(:,5);
T_pump = A_simout_TC.signals.values(:,1);
T_turbine = A_simout_TC.signals.values(:,2);
F_roll = A_simout_wheel.signals.values(:,1);
F_trac = A_simout_wheel.signals.values(:,2);
P_lift = A_simout_power_balance.signals.values(:,1);
P_trans = A_simout_power_balance.signals.values(:,2);
P_steering = A_simout_power_balance.signals.values(:,3);
P_loss_TC = A_simout_power_balance.signals.values(:,4);
% lifting system
v_lift = A_simout_lifting.signals.values(:,1);
H_bucket = A_simout_lifting.signals.values(:,2);
v_lift_max = A_simout_lifting.signals.values(:,3);
%% plot some interesting variables
if 1
    close all
    figure(1)
    yyaxis left
    plot(time,engine_speed./pi.*30);
    yyaxis right
    plot(time,Te)
    hold on
    grid on
    xlabel('time s')
    legend('\omega_{e} rpm','T_{e} Nm')
    hold off
    
    figure(2)
    plot(time,Pm,'o',time,P_im,'r.-');
    hold on
    grid on
    xlabel('time s')
    ylabel('Pressure Pa')
    legend('P_{stable}','P_{im}')
    hold off
    
    figure(3)
    plot(time,Te,'r.-',time,T_pump,'b',time,T_turbine,'y');
    hold on
    grid on
    xlabel('time s')
    legend('T_{e} Nm','T_{pump} Nm','T_{trubine} Nm')
    hold off
    %
    % %     figure(4)
    % %     %
    % %     %plot(t, Te .* engine_speed);
    % %
    % %     %yyaxis left
    % %     % plot(t, T_turbine .*gamma_gb .* V ./r_w);
    % %     % hold on
    % %     %yyaxis right
    % %     plot(t,P_loss_TC)
    % %
    % %     legend('P_{loss,TC}')
    % %     grid on
    %
    figure(5)
    subplot(211)
    plot(time,V)
    legend('vehicle speed m/s')
    grid on
    subplot(212)
    plot(time,F_roll,'ro-')
    hold on
    plot(time,F_trac,'b*-')
    grid on
    legend('F_{roll} N','F_{trac} N')
    hold off
    %
    %     figure(7)
    %     plot(t,1./R)
    %     legend('\tau = 1/R')
    %     grid on
    %
    
    figure(9)
    subplot(211)
    plot(time,v_lift,'y','linewidth',2)
    hold on
    plot(time,v_lift_max,'.r','linewidth',2)
    legend('V_{lift} m/s','V_{lift,max} m/s')
    grid on
    hold off
    subplot(212)
    plot(time,H_bucket,'b','linewidth',2)
    legend('H_{bucket} m/s')
    grid on
    
    figure(10)
    subplot(211)
    plot(time,P_trans,time,P_steering,time,P_lift)
    hold on
    P_e = Te .* engine_speed;
    plot(time,P_e,'r','linewidth',1.5)
    grid on
    legend('P_{trans}','P_{steering}','P_{lift}','P_e')
    
    subplot(212)
    P_out = P_trans + P_steering + P_lift;
    plot(time,P_e,time,P_out)
    grid on
    legend('P_e','P_{out}')
    
    figure(12)
    plot(X_position,Y_position,'r','linewidth',2.5);
    xlabel('X position m')
    ylabel('Y position m')
    grid on
    %
    %     figure(9)
    %     plot(t,rem(theta_heading,360),'r')
    %     hold on
    %     plot(t,gamma_driving,'g')
    %     %plot(t,gamma_driving,'b')
    %     legend('\theta_{heading}','\gamma_{driving}')
    %     %plot(t,theta_heading_1 ,'b--')
    %
    %     %fill([t fliplr(t)],[gamma_driving fliplr(rem(theta_heading,360))],'c');
    %     hold off
    %
    %     figure(10)
    %     plot(t,u_str./ 180 .* pi,'r:')
    %     legend('the control input: u_{str}')
    %
    %     figure(11)
    %     plot(input_database.minT.time,input_database.minT.delta_driving)
    %     hold on
    %     plot(t,gamma_driving,'g')
    %     legend('original','\gamma_{driving}')
end