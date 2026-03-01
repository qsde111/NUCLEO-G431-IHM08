 % clc;
 clear; close all;

PI = 3.14159265359;

bandwidth = 150;

Wn=2*PI*bandwidth;

zeta=1;

Kp=2*zeta*Wn;

Ki=Wn*Wn;

fprintf('Kp = %.0f\n', Kp);
fprintf('Ki = %.0f\n', Ki);