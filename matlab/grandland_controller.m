function u_d = grandland_controller(Kd_coeff, r_v, theta, theta_dot)

persistent state;
if isempty(state)
    state = zeros(4, 3);
end

state(:, 2:3) = state(:, 1:2);
state(1:3, 1) = [r_v; theta; theta_dot];

u_d = 0;
for i = 1:3
    num = Kd_coeff(i, 1:3);
    u_d = u_d+(num(1)*state(i, 1)+num(2)*state(i, 2)+num(3)*state(i, 3));
end
den = Kd_coeff(i+1, 1:3);
u_d = u_d-den(2)*state(4, 2)-den(3)*state(4, 3);

state(4, 1) = u_d;

end