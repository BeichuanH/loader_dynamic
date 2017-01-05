function [k_theta,theta_1] = K_theta(theta,r,r1,xc,yc)
%%% The lifting system 2014 version
%%% The Geometry coefficient of the lifting system
%%% k_theta is a ratio of bucket lifting and delta_length of hydrulic pumps
%% The parameters
% unit:m
% r = 2.9; % the boom length; 
% r1 = 1.7; % the boom length from body joint (O) to connection
% r2 = 1.2; % the boom lenght from connection to bucket
% G = 2.3; % the vertical distance from bucket to body joint
% xc = 0.19; % dimension horizontal axis
% yc = -0.3; % dimension vertical axis


%% inputs 
%theta = -40:0.01:80; %   from degree to radian:  .* pi ./ 180

%% The formulae
%theta_1 = theta_1 + (theta_1 <= 0) * pi;
L_cyl = sqrt((r1 * cos(theta .* pi ./ 180) - xc).^2 + (r1 * sin(theta .* pi ./ 180) - yc).^2);
delta_theta = diff(theta);
delta_theta = delta_theta + (delta_theta==0) * eps;
delta_L_cyl = diff(L_cyl);
%delta_L_cyl = delta_L_cyl + (delta_L_cyl==0) * eps;
div_delta_L = delta_L_cyl ./ delta_theta;
%div_delta_L = div_delta_L + (abs(div_delta_L)<=.0001) * 0.001;
k_theta = (diff(r .* sin(theta .* pi ./ 180)) ./ delta_theta)./ (div_delta_L);
k_theta(end+1) = k_theta(end); 


%% parameters checking
theta_1 = (atan((r1 * sin(theta .* pi ./ 180) - yc) ./ (r1 * cos(theta .* pi ./ 180 ) - xc))).* 180 ./ pi;
if 0
figure(1)
plot(theta(1:end-1),(diff(r .* sin(theta .* pi ./ 180)) ./ delta_theta))
hold on;
plot(theta(1:end-1),(div_delta_L))
xlabel('\theta_{2} [degree]')
legend('numerator: \Delta (r \bullet sin \theta ) / \Delta \theta','denominator: \Delta L_{cyl} / \Delta \theta')
hold off;

figure(2)
plot(theta,k_theta)
xlabel('\theta_{2} [degree]')
ylabel('K(\theta_{2})')

figure(3)
plot(theta,theta_1)
xlabel('\theta_{2} [degree]')
ylabel('\theta_{1} [degree]')

figure(4)
plot((r * cos(theta .* pi ./ 180) - xc),(r * sin(theta .* pi ./ 180) - yc),'b');
xlabel('horizontal positon of the bucket: x')
ylabel('vertical position of the bucket: y')
legend('the curve of the bucket')
end