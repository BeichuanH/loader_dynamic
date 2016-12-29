% powertrain system
clear;%clc;

%% Time consumption
load input_database;

% time consumption for short loading path
t = input_database.minT.time;
size_t = size(t); % using variable to store the size of running time: t
delta_t = 0.1;

%% control inputs
% control inputs
u_f = input_database.minT.mf; % injected fuel per cycle unit:mg/cycle
u_str = input_database.minT.steer .* 180 ./pi;
u_b = input_database.minT.break;
u_ab = input_database.minT.ab;
% state variables for standard results
V_vehicle = input_database.minT.V; %the state variavles : speed of the vechile unit:m/s


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

%% Initialization of state variables of the engine and powertrain system
% engine system
engine_speed = ones(size_t);
engine_speed(1) = 1432.4./60 .* 2 .* pi; % unit: rad/s
eratio_speed = zeros(size_t);
eratio_speed(1) = engine_speed(1) ./ 1000;

T_ig = zeros(size_t);
T_fric = zeros(size_t);
Te = zeros(size_t);
Te(1) = 1670;

T_ig(1) = eta_ig .* q_hv .* n_cyl .* u_f(1) .*(10^-6) ./ (4 .* pi); %the calculation of indicted torque
T_fric(1) = Vd * 1e5 ./ (4 * pi) .*(c_fri_1 .* eratio_speed(1).^2 + c_fri_2 .* eratio_speed(1) + c_fri_3); %the calculation of engine friction torque
Te(1) = T_ig(1)  - T_fric(1);

Pm = zeros(size_t);
Pm(1) = 1.5e5; % unit: kPa
P_im = 1.3e5 .* ones(size_t);
P_im(1) = 1.3e5; % unit: kPa
tau_m = ones(size_t);
m_air = zeros(size_t);
m_air(1) = 200000;
m_f = zeros(size_t);
lambda = zeros(size_t);
u_f_limitation = zeros(size_t);

% torque converter
gamma_gb = 60 .* ones(size_t);
omega_gb = ones(size_t);
phi = ones(size_t);
T_pump = ones(size_t);
T_turbine = ones(size_t);
T_w = ones(size_t);
kappa = ones(size_t);
ephsilon = ones(size_t);
% vehilce moving
F_roll = zeros(size_t);
F_trac = zeros(size_t);
V = zeros(size_t);
V(1) = -0.02;

% steering system
gamma_driving = zeros(size_t); % unit:degree
R =Inf .* ones(size_t);
%ground position of the vehicle: (x,y,theta)
theta_heading = 90 * ones(size_t); % unit:degree
x_position = 25 * ones(size_t); % unit:m
y_position = 40 * ones(size_t); % unit:m

% lifting system
%% Generate the geometry efficiency
theta = -40:0.01:80;
[k_theta,theta_1] = K_theta(theta,r,r1,xc,yc);

%% fluid pump limitation
% engine_speed = 1500./60; %unit: rps
% Q_pump_max(i) = min(engine_speed(i),157) .* D_pump .* eta_volumetric .* 10^-3;
% v_cylinder_max(i) = Q_pump_max(i) .* n_cyl_hydra ./ A_cylinder;

Q_pump_max = zeros(size_t);
Q_pump_max(1) = min(engine_speed(1),157) .* D_pump .* eta_volumetric .* 10^-3;
v_cylinder_max = zeros(size_t);
v_cylinder_max(1) = Q_pump_max(1) .* n_cyl_hydra ./ A_cylinder;
% power balance
P_trans = zeros(size_t);
P_steering = zeros(size_t); % steering power; unit:W
T_out = zeros(size_t);
T_out(1) = 1649;

V_lift = zeros(size_t);
v_cylinder = zeros(size_t);
V_lift_max = zeros(size_t); %the maximun speed of the lifting boom
theta_t = zeros(size_t);
theta_t(1) = -20;
k_theta_t = zeros(size_t);
theta_1_t = zeros(size_t);
F_cyl = zeros(size_t);
H_bucket_t = zeros(size_t);
H_bucket_t(1) = r * sin(theta_t(1) .* pi ./ 180);
L_cyl = zeros(size_t);
L_cyl(1) = sqrt((r1 * cos(theta_t(1) .* pi ./ 180) - xc).^2 + (r1 * sin(theta_t(1) .* pi ./ 180) - yc).^2);
F_load = zeros(size_t);
P_lift = zeros(size_t);



%% the simulation of the powertrain running
for i = 2:length(t)
    %% the calculation of engine torque
    if u_f_limitation(i-1) ~= 0  && u_f(i) >= u_f_limitation(i-1)
        u_f(i) = u_f_limitation(i-1);
    end
    eratio_speed(i-1) = engine_speed(i-1) ./ 1000;
    T_ig(i) =  eta_ig .* q_hv .* n_cyl .* u_f(i) .*(10^-6) ./ (4 .* pi); %the calculation of indicted torque
    T_fric(i) = Vd * 1e5 ./ (4 * pi) .*(c_fri_1 .* eratio_speed(i-1).^2 + c_fri_2 .* eratio_speed(i-1) + c_fri_3); %the calculation of engine friction torque
    Te(i) = T_ig(i)  - T_fric(i);
    %% the turbochanger system (1st order delay)
    Pm(i) = c_p1 .* (engine_speed(i-1)).^2 + c_p2 .* Te(i) + c_p4 .* (Te(i) .* engine_speed(i-1)) + c_p3;
    Pm(i) = Pm(i) .* 9e-5;  % after 2 days guess, have to use a trick for controlling the value in the reasonable area.
    if ge(Pm(i) , 4.5e5)
        Pm(i) = 4.5e5;
    end
    tau_m(i) = c_t1 .* (engine_speed(i-1)).^c_t2;
    P_im(i) = P_im(i-1) + 1 ./ tau_m(i) .* (Pm(i) - P_im(i-1)) .* delta_t;
    
    %% the smoke limits
    m_air(i) = eta_vol_eng .* Vd .* engine_speed(i-1) .* 60 ./ pi ./2 .* P_im(i) ./ (4 .* pi .* Ra .* T_amb); % uint: kg/s
    m_f(i) = 1e-6 ./ (4 .* pi) .* u_f(i) .* engine_speed(i-1) .* 60 ./ pi ./2 .* n_cyl; % unit: kg/s
    if u_f(i) ~= 0
        lambda(i) = ( m_air(i) ./ m_f(i) ) ./ 14.7; % 14.7 is the stoichiometric air to fuel ratio, and lambda is the air to fuel ratio indicator
        if lambda(i) <= lambda_minimum
            u_f_limitation(i) = (m_air(i) ./ (lambda_minimum .* 14.7)) ./ (1e-6 ./ (4 .* pi) .* engine_speed(i-1) .* n_cyl .* 60 ./ pi ./2);
        end
    end
    engine_speed(i) = engine_speed(i-1) + 1./I_ice .* (Te(i) - T_out(i-1)) .* delta_t; %the engine rotation
    
    if ge(engine_speed(i),230.38)
        engine_speed(i) = 230.38;
    elseif le(engine_speed(i),57)
        engine_speed(i) = 57;
    end
    
    %% vehicle powertrain system for moving
    % the powertrain transfer
    if sign(V(i-1)) == 1
        gamma_gb(i) = gear_ratio(3);
    elseif sign(V(i-1)) == -1
        gamma_gb(i) = gear_ratio(1);
    else
        gamma_gb(i) = gear_ratio(2);
    end
    omega_gb(i) = V(i-1) .* gamma_gb(i) ./ r_w;
    phi(i) = omega_gb(i) ./ engine_speed(i);
    [ephsilon(i),kappa(i)] = TC_search(phi(i));
    T_pump(i) = ephsilon(i) .* (engine_speed(i) .*60 ./ (2 .* pi) ./1000).^2;
    T_turbine(i) = kappa(i) .* (engine_speed(i) .*60 ./ (2 .* pi) ./1000).^2;
    T_w(i) = T_turbine(i) .* eta_gb .* gamma_gb(i);
    % the vehicle moving
    F_roll(i) = sign(V(i-1)) .* c_r .* (M_veh + M_buc) .* 9.8;
    F_trac(i) = (T_w(i) - sign(V(i-1)) .* u_b(i)) ./ r_w;
    V(i) = V(i-1) + (- F_roll(i) + F_trac(i)) ./ (M_buc + M_veh + M_wheels) .* delta_t;
    if sign(V(i-1)) ~= sign(V(i))
        u_b(i : i+10) = 0;
    end
    
    %% steering system for moving
    if ge(u_str(i), u_str_max)
        u_str(i) = u_str_max;
    elseif le(u_str(i), u_str_min)
        u_str(i) = u_str_min;
    end
    gamma_driving(i) = gamma_driving(i-1) + u_str(i) .* delta_t;
    if ge(gamma_driving(i),20)
        gamma_driving(i) = 20;
    elseif le(gamma_driving(i),-20)
        gamma_driving(i) = -20;
    end
    R(i) = L ./ (2 .* (tan((gamma_driving(i)) .* 0.5 .* pi ./ 180)));
    if ge(R(i),50)
        R(i) = 50;
    elseif le(R(i),-50)
        R(i) = -50;
    end
    theta_heading(i) = theta_heading(i-1) + V(i-1) ./ R(i) ./ pi .*180 .* delta_t;
    x_position(i) = x_position(i-1) + V(i-1) .* cos(theta_heading(i) .* pi ./ 180) * delta_t;
    y_position(i) = y_position(i-1) + V(i-1) .* sin(theta_heading(i) .* pi ./ 180) * delta_t;
    
    %% lifting system
    [k_theta_t(i-1),theta_1_t(i-1)] = K_theta_finding(theta_t(i-1),theta,k_theta,theta_1);
    
    Q_pump_max(i) = min(engine_speed(i),157) .* D_pump .* eta_volumetric .* 10^-3;
    v_cylinder_max(i) = Q_pump_max(i) .* n_cyl_hydra ./ A_cylinder;
    
    V_lift_max(i) = k_theta_t(i-1) * v_cylinder_max(i);
    V_lift(i) = V_lift(i-1) + delta_t .* u_ab(i-1);
    v_cylinder(i) = V_lift(i) ./ k_theta_t(i-1);
    
    
    if v_cylinder(i) > abs(v_cylinder_max(i)) || abs(V_lift(i)) > abs(V_lift_max(i)) 
        V_lift(i) = V_lift_max(i);
        v_cylinder(i) = V_lift(i) ./ k_theta_t(i-1);
        u_ab(i-1) = (V_lift_max(i) - V_lift(i-1))./delta_t;
    end
    
    % the changes of geometry variables
    L_cyl(i) = L_cyl(i-1) + v_cylinder(i) .* delta_t;
    H_bucket_t(i) = H_bucket_t(i-1) + V_lift(i) .* delta_t;
    theta_t(i) = abs(asin(H_bucket_t(i) ./ r))  .* 180 ./ pi;

    if theta_t(i) < min(theta)
        theta_t(i) = min(theta);
    elseif theta_t(i) > max(theta)
        theta_t(i) = max(theta);
    end
    
    F_cyl(i) = ((M_boom .* 1.5 + M_load) * r * u_ab(i) + (M_load + 0.5 .* M_boom) * 9.8 * r * cos(theta_t(i-1).* pi ./ 180)) ./ ...
        (r1 .* sin((theta_1_t(i-1) - theta_t(i-1)).* pi ./ 180));
    F_load(i) = M_load .* (9.8 + u_ab(i)); % unit:N
    
    
    %% power balance for calculating the output torque
    P_lift(i) = F_load(i) .* V_lift(i) ./ eta_lift; % unit:W
    P_trans(i) = T_pump(i) .* engine_speed(i);
    P_steering(i) = (u_str(i) ./ 180 .* pi).^2 .* c_p; % steering power; unit:W
    T_out(i) = (P_trans(i) + P_steering(i) + P_lift(i))./ engine_speed(i);
    
end

%% system evaluations

P_loss_TC = P_trans - T_turbine .*gamma_gb .* V ./r_w; 



%% Checking system
if 1
    close all
    figure(1)
    plot(t,engine_speed./pi.*30,'b.',t,Te,'r.-');
    hold on
    grid on
    xlabel('time s')
    legend('\omega_{e} rpm','T_{e} Nm')
    hold off
    
    
    figure(2)
    plot(t,Pm,'o',t,P_im,'r.-');
    hold on
    grid on
    xlabel('time s')
    ylabel('Pressure Pa')
    legend('P_{stable}','P_{im}')
    hold off
    
    figure(3)
    plot(t,Te,'r.-',t,T_pump,'b',t,T_turbine,'y');
    hold on
    grid on
    xlabel('time s')
    legend('T_{e} Nm','T_{pump} Nm','T_{trubine} Nm')
    hold off
    
%     figure(4)
%     %yyaxis left
%     %plot(t, Te .* engine_speed);
%     
%     %yyaxis left
%     % plot(t, T_turbine .*gamma_gb .* V ./r_w);
%     % hold on
%     %yyaxis right
%     plot(t,P_loss_TC)
%     
%     legend('P_{loss,TC}')
%     grid on
    
    figure(5)
    plot(t,V)
    legend('vehicle speed m/s')
    grid on
    
    figure(6)
    plot(t,F_roll,'ro-')
    hold on
    plot(t,F_trac,'b*-')
    grid on
    legend('F_{roll}','F_{trac}')
    hold off
    
%     figure(7)
%     plot(t,1./R)
%     legend('\tau = 1/R')
%     grid on
%     
    figure(8)
    plot(x_position,y_position,'k*');
    grid on
    
    figure(10)
    plot(t,P_trans,t,P_steering,t,P_lift)
    hold on
    P_e = Te .* engine_speed;
    plot(t,P_e)
    grid on
    legend('P_{trans}','P_{steering}','P_{lift}','P_e')
    
    figure(11)
    P_out = P_trans + P_steering + P_lift;
    plot(t,P_e,t,P_out)
    grid on
    legend('P_e','P_{out}')
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