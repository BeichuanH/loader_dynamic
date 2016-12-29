function [k_theta_t,theta_1_t] = K_theta_finding(theta_t,theta,k_theta,theta_1)
%%% The lifting system 2014 version
%%% The Geometry coefficient of the lifting system
%%% k_theta is a ratio of bucket lifting and delta_length of hydrulic pumps
flag=0;
for i = 1:length(theta)-1
    if (theta(i)<=theta_t) && (theta(i+1)>=theta_t)
        k_theta_t = 0.5 * (k_theta(i) + k_theta(i+1));
        theta_1_t = 0.5 * (theta_1(i) + theta_1(i+1));
        flag = 1;
    end
end
if flag==0
    error('Theta out of range')
end

