clear, clc, close all;
s = tf('s');
t = 0:0.001:2;

%% PARAMETERS
r = 0.033;
mb = 0.65;
mw = 0.028;
g = 9.81;
lg = 0.022;
Ib = 8.824e-4;
Iw = mw*r^2/2;
m = mb+2*mw;

M = [mb+2*mw+2*Iw/r^2 -mb*lg;
    -mb*lg Ib+mb*lg^2];
ep = [0 0;
    0 -mb*lg*g];
H = [1/r; 1];

%% ROBOT
G = -(2*Iw+m*lg*r)*s/((Ib+m*lg*r)*s^2-mb*lg*g);

wc = getGainCrossover(G, getPeakGain(G)/sqrt(2));
wc = wc(end);
Ts_max = 1/wc;
Ts = 0.01;

zoh = 1/(s*Ts/2+1);

K = pidtune(zoh*G, 'PIDF');

%[K, ~, GAM] = mixsyn(zoh*G, [], [], 1)
cl1 = loopsens(zoh*G, K);
step(cl1.To);
% 
% R = reducespec(K, 'balanced');
% rK = getrom(R, Order = 1);
% cl2 = loopsens(zoh*Wu, rK);
% 
% Kd = c2d(K, Ts, 'tustin');

%%
% [num, den] = tfdata(Kd, 'v');
% Kd_coeff = [num; den];