%  Aidan Fitzpatrick - ROB-GY 7863 AMR Project
%  4-Wheeled Anti-Ackermann (RWD) Car - Dynamics Validation
% [X; Y; theta; Vx; Vy; omega]  Front wheels steering only | Rear wheels: force only

clc;
clear all;
close all;


m = 700; % mass (kg)
Izz = 800; % inertia (kg*m^2)
l = 3.6/2; % half wheelbase (m)
b = 2.0/2; % half track



X0 = 0;
Y0 = 0;
theta0 = 0;
Vx0 = 5; % initial forward speed (m/s)
Vy0 = 0;
omega0 = 0;

% State
x0 = [X0; Y0; theta0; Vx0; Vy0; omega0];

tspan = [0 10];% 10 seconds

[t, X] = ode45(@(t,x) dynamics(t,x,m,Izz,l,b), tspan, x0);

Xpos = X(:,1);
Ypos = X(:,2);
theta = X(:,3);
Vx = X(:,4);
Vy = X(:,5);
omega = X(:,6);

delta1 = zeros(size(t)); % FR
delta2 = zeros(size(t)); % FL
delta3 = zeros(size(t)); % RR
delta4 = zeros(size(t)); % RL

for k = 1:length(t)
    base = deg2rad(15)*sin(0.5*t(k));  % smooth sinusoidal steering

    if base >= 0
        delta2(k) = base; % FL (inside)
        delta1(k) = 1.2*base; % FR (outside)
    else
        delta1(k) = base; % FR (inside)
        delta2(k) = 1.2*base; % FL (outside)
    end

    delta3(k) = 0;
    delta4(k) = 0;
end




figure;
plot(Xpos, Ypos,'LineWidth',2);
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Trajectory');

figure;
subplot(3,1,1);
plot(t,Vx,'LineWidth',1.5);
ylabel('V_x (m/s)'); grid on;
title('Body-frame Velocities and omega');

subplot(3,1,2);
plot(t,Vy,'LineWidth',1.5);
ylabel('V_y (m/s)'); grid on;

subplot(3,1,3);
plot(t,omega,'LineWidth',1.5);
ylabel('\omega (rad/s)'); xlabel('t (s)'); grid on;

figure;
plot(t,rad2deg(delta1),'LineWidth',1.5); hold on;
plot(t,rad2deg(delta2),'LineWidth',1.5);
plot(t,rad2deg(delta3),'--','LineWidth',1.5);
plot(t,rad2deg(delta4),'--','LineWidth',1.5);
xlabel('t (s)'); ylabel('\delta_i (deg)');
title('Steering Angles (Anti-Ackermann)');
legend('\delta_1 FR','\delta_2 FL','\delta_3 RR','\delta_4 RL');
grid on;






%  DYNAMICS FUNCTION (Simple lateral tire forces to approximate no-slip)
function dx = dynamics(t, x, m, Izz, l, b)
    % state: x = [X; Y; theta; Vx; Vy; omega]

    X = x(1);
    Y = x(2);
    theta = x(3);
    Vx = x(4);
    Vy = x(5);
    omega = x(6);

    % Same as main script
    base = deg2rad(15)*sin(0.5*t);

    if base >= 0
        delta2 = base; % FL (inside)
        delta1 = 1.2*base;% FR (outside)
    else
        delta1 = base; % FR (inside)
        delta2 = 1.2*base; % FL (outside)
    end

    delta3 = 0; % RR
    delta4 = 0; % RL

    deltas = [delta1; delta2; delta3; delta4];

    % wheel pos's
    xw = [ +l; +l; -l; -l];
    yw = [ -b; +b; -b; +b];

    F_drive = 4000; % N per wheel
    Flong   = [0; 0; F_drive; F_drive];




    % slip velo's at each wheel
    % Compute velocity at each wheel in body frame and put in wheel frame
    v_long = zeros(4,1);
    v_lat  = zeros(4,1);

    for i = 1:4
        % Velocity of wheel in body frame
        vx_i = Vx - omega*yw(i);
        vy_i = Vy + omega*xw(i);

        % wheel frame (aligned with delta_i)
        v_long(i) =  vx_i*cos(deltas(i)) + vy_i*sin(deltas(i));
        v_lat(i)  = -vx_i*sin(deltas(i)) + vy_i*cos(deltas(i));
    end

    % Lateral tire forces (no-slip approx) ~ Simple linear lateral stiffness
    k_lat = 80000; % (N/(m/s))
    Flat = -k_lat*v_lat;

    % wheel frame -> body frame:
    % [Fx_i; Fy_i] = R(delta_i) * [Flong_i; Flat_i]
    Fx = Flong.*cos(deltas) - Flat.*sin(deltas);
    Fy = Flong.*sin(deltas) + Flat.*cos(deltas);

    % Yaw torques from each wheel
    % tau_i = x_i * Fy_i - y_i * Fx_i
    tau_z = xw.*Fy - yw.*Fx;

    % Net body wrench
    Fxb = sum(Fx);
    Fyb = sum(Fy);
    Mz = sum(tau_z);

    %Dynamcsi
    % m*Vx_dot - m*omega*Vy = Fxb
    % m*Vy_dot + m*omega*Vx = Fyb
    Vx_dot = (Fxb/m) + omega*Vy;
    Vy_dot = (Fyb/m) - omega*Vx;
    omega_dot = Mz / Izz;

    % put in global frame
    X_dot = Vx*cos(theta) - Vy*sin(theta);
    Y_dot = Vx*sin(theta) + Vy*cos(theta);
    theta_dot = omega;

    dx = [X_dot;Y_dot;theta_dot;Vx_dot;Vy_dot;omega_dot];
end