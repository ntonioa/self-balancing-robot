clear, clc, close all;

s = tf('s');
l = 0.3;
g = 9.81;

G1 = -s/(s^2*l-g);
K1 = pidtune(G1, 'PIDF');

cl1 = loopsens(G1, K1);
impulse(cl1.To);

G2 = cl1.CSo;
K2 = pidtune(G2, 'PIDF');

cl2 = loopsens(G2, K2);
step(cl2.To);

G1.u = 'v';
G1.y = '\theta';
K1.u = 'e_{\theta}';
K1.y = 'v';
K2.u = 'e_v';
K2.y = 'r_{\theta}';
sum_v = sumblk('e_v = r_v-v');
sum_th = sumblk('e_{\theta} = r_{\theta}-\theta');

N = connect(G1, K1, K2, sum_v, sum_th, 'r_v', {'\theta', 'v'});
step(N)

