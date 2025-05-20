% Parameters for the mass-damper system
m1 = 0.225;  % Mass of the damper (kg)
k1 = 9.588;  % Stiffness of the damper (N/m)
c1 = 0.5;    % Damping coefficient of the damper (Ns/m)

m2 = 1.14;   % Mass of the building (kg)
k2 = 38.62;  % Stiffness of the building (N/m)
c2 = 1.0;    % Damping coefficient of the building (Ns/m)

% Time vector
t = linspace(0, 10, 1000);  % Simulate for 10 seconds

% Initial conditions [x1, v1, x2, v2]
initial_conditions = [0.1; 0; 0.05; 0]; % Initial displacement and velocity

% Define system of ODEs for coupled oscillation
coupled_ode = @(t, y) [
    y(2);
    (-c1 * y(2) - k1 * y(1) + c2 * (y(4) - y(2)) + k2 * (y(3) - y(1))) / m1;
    y(4);
    (-c2 * (y(4) - y(2)) - k2 * (y(3) - y(1))) / m2
];

% Solve the system using ode45
[t, sol] = ode45(coupled_ode, t, initial_conditions);

% Extract positions and velocities
x1 = sol(:, 1); % Displacement of damper
v1 = sol(:, 2); % Velocity of damper
x2 = sol(:, 3); % Displacement of building
v2 = sol(:, 4); % Velocity of building

% Energy calculations
KE1 = 0.5 * m1 * v1.^2; % Kinetic energy of damper
PE1 = 0.5 * k1 * x1.^2; % Potential energy of damper
ME1 = KE1 + PE1;        % Mechanical energy of damper

KE2 = 0.5 * m2 * v2.^2; % Kinetic energy of building
PE2 = 0.5 * k2 * x2.^2; % Potential energy of building
ME2 = KE2 + PE2;        % Mechanical energy of building

% Dissipated energy (simplified damping loss model)
diss_energy1 = c1 * v1.^2;
diss_energy2 = c2 * v2.^2;

% Plot results
figure;

% Displacement plot
subplot(4,1,1);
plot(t, x1, 'r', 'DisplayName', 'Damper');
hold on;
plot(t, x2, 'b', 'DisplayName', 'Building');
title('Theoretical Displacement Over Time');
xlabel('Time [s]');
ylabel('Displacement [m]');
legend;
grid on;

% Energy in the building
subplot(4,1,2);
plot(t, KE2, 'r', 'DisplayName', 'Kinetic Energy');
hold on;
plot(t, PE2, 'b', 'DisplayName', 'Potential Energy');
plot(t, ME2, 'k', 'DisplayName', 'Mechanical Energy');
title('Building Energy Over Time');
xlabel('Time [s]');
ylabel('Energy [J]');
legend;
grid on;

% Energy in the damper
subplot(4,1,3);
plot(t, KE1, 'r', 'DisplayName', 'Kinetic Energy');
hold on;
plot(t, PE1, 'b', 'DisplayName', 'Potential Energy');
plot(t, ME1, 'k', 'DisplayName', 'Mechanical Energy');
title('Damper Energy Over Time');
xlabel('Time [s]');
ylabel('Energy [J]');
legend;
grid on;

% Dissipated energy
subplot(4,1,4);
plot(t, diss_energy1, 'r', 'DisplayName', 'Damper');
hold on;
plot(t, diss_energy2, 'b', 'DisplayName', 'Building');
title('Dissipated Energy Over Time');
xlabel('Time [s]');
ylabel('Energy [J]');
legend;
grid on;

sgtitle('Coupled Mass Damper System Simulation');


