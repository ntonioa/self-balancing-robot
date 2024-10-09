clear, clc, close all;
s = tf('s');
t = 0:0.001:2;

%% PARAMETERS
r = 0.033;
mb = 0.65;
mw = 0.028;
g = 9.81;
lg = 0.022;
% Ib = mb*(2*lg)^2;
Ib = 8.824e-4;
Iw = mw*r^2/2;

M = [mb+2*mw+2*Iw/r^2 -mb*lg;
    -mb*lg Ib+mb*lg^2];
ep = [0 0;
    0 -mb*lg*g];
H = [1/r; 1];

%% ROBOT
A = [0 0 1 0;
    0 0 0 1;
    -M\ep zeros(2)];
B = [0;
    0;
    M\H];
C = [0 1 0 0;
    0 0 0 1;
    0 0 1/r 1];
G = ss(A, B, C, 0);

%% 
% Ktau = pidtune(G(3), 'PIDF');
Ktau = tf(1.85);

G.u = '\tau';
G.y = {'\theta', 'd\theta/dt', '\alpha'};
Ktau.u = 'e_u';
Ktau.y = '\tau';
err_u = sumblk('e_u = u-\alpha');

Wu = connect(G, Ktau, err_u, 'u', {'\theta', 'd\theta/dt'});
%Wu = [G(1); G(2)]/G(3);

wc = getGainCrossover(Wu(1), getPeakGain(Wu(1))/sqrt(2));
wc = wc(end);
Ts_max = 1/wc;
Ts = 0.02;
%% CONTROL SYSTEM
zoh = 1/(s*Ts/2+1);
Ku = pidtune(Wu(1)*zoh, 'PI');

Wu.u = 'u';
Wu.y = {'\theta', 'd\theta/dt'};
zoh.u = 'u_d';
zoh.y = 'u';
Ku.u = 'e_\theta';
Ku.y = 'u_d';
err_theta = sumblk('e_\theta = r_\theta-\theta');

Wtheta = connect(Wu, zoh, Ku, err_theta, 'r_\theta', {'\theta', 'd\theta/dt', 'u'});

%%
phi = [-r r];
phi = tf(phi);
Wtheta_phi = phi*Wtheta(2:3);
Ktheta = pidtune(Wtheta_phi, 'PIDF');
Ktheta = -8.5404*(s+0.03511)/(s*(s+8.519));

Wtheta.u = 'r_\theta';
Wtheta.y = {'\theta', 'd\theta/dt', 'u'};
phi.u = {'d\theta/dt', 'u'};
phi.y = 'v';
Ktheta.u = 'e_v';
Ktheta.y = 'r_\theta';
err_v = sumblk('e_v = r_v-v');

Wv = connect(Wtheta, phi, Ktheta, err_v, 'r_v', {'\theta', 'd\theta/dt', 'u', 'v'});

%%
figure;

lsim(Wv, 0.1*(t>=0), t)

%% CONTROLLER
K = connect(phi, err_v, Ktheta, err_theta, Ku, zoh, {'r_v', '\theta', 'd\theta/dt'}, 'u_d');

R = reducespec(K, 'balanced');
rK = getrom(R, Order = 2);

Kd = c2d(rK, Ts, 'tustin');

Kd_coeff = zeros(4, 3);
for i = 1:3
    [num, den] = tfdata(Kd(i), 'v');
    Kd_coeff(i, 1:3) = num;
end
 Kd_coeff(i+1, 1:3) = den;