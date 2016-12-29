
function [varepsilon,kappa] = TC_search(Phi)
% Linear model Poly6:
%      f(x) = p1*x^6 + p2*x^5 + p3*x^4 + p4*x^3 + p5*x^2 + 
%                     p6*x + p7
% Coefficients (with 95% confidence bounds):
%        p1 =      -57.24  (-107, -7.468)
%        p2 =        -177  (-479, 125)
%        p3 =        1339  (639.2, 2039)
%        p4 =       -1794  (-2566, -1022)
%        p5 =       334.3  (-75.2, 743.7)
%        p6 =      -39.47  (-131.2, 52.23)
%        p7 =       406.2  (400.1, 412.3)
% 
% Goodness of fit:
%   SSE: 2844
%   R-square: 0.9996
%   Adjusted R-square: 0.9996
%   RMSE: 5.415
x = Phi;
       p1 =      -57.24;  %(-107, -7.468)
       p2 =        -177;  %(-479, 125)
       p3 =        1339;  %(639.2, 2039)
       p4 =       -1794;  %(-2566, -1022)
       p5 =       334.3;  %(-75.2, 743.7)
       p6 =      -39.47;  %(-131.2, 52.23)
       p7 =       406.2;  %(400.1, 412.3)
 varepsilon = p1*x^6 + p2*x^5 + p3*x^4 + p4*x^3 + p5*x^2 + p6*x + p7;
 
 
% Linear model Poly6:
%      f(x) = p1*x^6 + p2*x^5 + p3*x^4 + p4*x^3 + p5*x^2 + 
%                     p6*x + p7
% Coefficients (with 95% confidence bounds):
%        p1 =       26.26  (12.45, 40.06)
%        p2 =      -176.9  (-259.9, -93.83)
%        p3 =       427.4  (236.6, 618.1)
%        p4 =        -341  (-549.3, -132.8)
%        p5 =      -9.083  (-118.5, 100.3)
%        p6 =      -598.1  (-622.5, -573.7)
%        p7 =       675.4  (673.8, 677)
% 
% Goodness of fit:
%   SSE: 974.7
%   R-square: 1
%   Adjusted R-square: 1
%   RMSE: 2.129
kappa_p1 =       26.26;%  (12.45, 40.06)
kappa_p2 =      -176.9;%  (-259.9, -93.83)
kappa_p3 =       427.4;%  (236.6, 618.1)
kappa_p4 =        -341;%  (-549.3, -132.8)
kappa_p5 =      -9.083;%  (-118.5, 100.3)
kappa_p6 =      -598.1;%  (-622.5, -573.7)
kappa_p7 =       675.4;%  (673.8, 677)
kappa = kappa_p1*x^6 + kappa_p2*x^5 + kappa_p3*x^4 + kappa_p4*x^3 + kappa_p5*x^2 + kappa_p6*x + kappa_p7;
