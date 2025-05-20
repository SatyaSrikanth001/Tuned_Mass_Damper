% Case 1: Pendulum Moves (Dynamic)
% Inputs for the pendulum case
L = input('What is the length of the pendulum rod? '); % Length of pendulum rod
t_p(1) = input('What is the initial condition for t? '); % Initial time
X_p(1) = input('What is the initial displacement of the top? '); % Initial displacement of the top

% Constants
p = 0.050;   % Mass of pendulum bob
m = 0.198;   % Mass of large mass
g = 9.81;    % Gravitational acceleration
k = 15;      % Large mass spring constant
c = 0.124;   % Large mass damping coefficient
b = 0.00022;

% Equations for the pendulum system
fX_p = @(t, X, Q, V, W) V;
fQ_p = @(t, X, Q, V, W) W;
fV_p = @(t, X, Q, V, W) (p*L*(W^2)*sin(Q) + p*g*sin(Q)*cos(Q) - k*X - c*V + (b/L)*W*cos(Q)) / ((m + p*(sin(Q))^2));
fW_p = @(t, X, Q, V, W) (-p*L*(W^2)*sin(Q)*cos(Q) - (p + m)*g*sin(Q) + k*X*cos(Q) + c*V*cos(Q) - (1 + m/p)*(b/L)*W) / (L*(m + p*(sin(Q))^2));
% Initial conditions
Q(1) = 0; % Initial angular displacement
V_p(1) = 0; % Initial velocity
W(1) = 0; % Initial angular velocity

% Step size and number of steps
h = 0.0001;
time_total = 20;
n = ceil(time_total / h);

% RK4 loop for pendulum moving case
for i = 1:n
    t_p(i+1) = t_p(i) + h;
    
    % RK4 for the pendulum system
    k1X = fX_p(t_p(i), X_p(i), Q(i), V_p(i), W(i));
    k1Q = fQ_p(t_p(i), X_p(i), Q(i), V_p(i), W(i));
    k1V = fV_p(t_p(i), X_p(i), Q(i), V_p(i), W(i));
    k1W = fW_p(t_p(i), X_p(i), Q(i), V_p(i), W(i));
    
    k2X = fX_p(t_p(i)+h/2, X_p(i)+h/2*k1X, Q(i)+h/2*k1Q, V_p(i)+h/2*k1V, W(i)+h/2*k1W);
    k2Q = fQ_p(t_p(i)+h/2, X_p(i)+h/2*k1X, Q(i)+h/2*k1Q, V_p(i)+h/2*k1V, W(i)+h/2*k1W);
    k2V = fV_p(t_p(i)+h/2, X_p(i)+h/2*k1X, Q(i)+h/2*k1Q, V_p(i)+h/2*k1V, W(i)+h/2*k1W);
    k2W = fW_p(t_p(i)+h/2, X_p(i)+h/2*k1X, Q(i)+h/2*k1Q, V_p(i)+h/2*k1V, W(i)+h/2*k1W);
    
    k3X = fX_p(t_p(i)+h/2, X_p(i)+h/2*k2X, Q(i)+h/2*k2Q, V_p(i)+h/2*k2V, W(i)+h/2*k2W);
    k3Q = fQ_p(t_p(i)+h/2, X_p(i)+h/2*k2X, Q(i)+h/2*k2Q, V_p(i)+h/2*k2V, W(i)+h/2*k2W);
    k3V = fV_p(t_p(i)+h/2, X_p(i)+h/2*k2X, Q(i)+h/2*k2Q, V_p(i)+h/2*k2V, W(i)+h/2*k2W);
    k3W = fW_p(t_p(i)+h/2, X_p(i)+h/2*k2X, Q(i)+h/2*k2Q, V_p(i)+h/2*k2V, W(i)+h/2*k2W);
    
    k4X = fX_p(t_p(i)+h, X_p(i)+h*k3X, Q(i)+h*k3Q, V_p(i)+h*k3V, W(i)+h*k3W);
    k4Q = fQ_p(t_p(i)+h, X_p(i)+h*k3X, Q(i)+h*k3Q, V_p(i)+h*k3V, W(i)+h*k3W);
    k4V = fV_p(t_p(i)+h, X_p(i)+h*k3X, Q(i)+h*k3Q, V_p(i)+h*k3V, W(i)+h*k3W);
    k4W = fW_p(t_p(i)+h, X_p(i)+h*k3X, Q(i)+h*k3Q, V_p(i)+h*k3V, W(i)+h*k3W);
    
    % Summing
    X_p(i+1) = X_p(i) + (h/6)*(k1X + 2*k2X + 2*k3X + k4X);
    Q(i+1) = Q(i) + (h/6)*(k1Q + 2*k2Q + 2*k3Q + k4Q);
    V_p(i+1) = V_p(i) + (h/6)*(k1V + 2*k2V + 2*k3V + k4V);
    W(i+1) = W(i) + (h/6)*(k1W + 2*k2W + 2*k3W + k4W);
end

% Case 2: Static Pendulum
% The pendulum is locked with no angular motion
X_s(1) = X_p(1); % Initial displacement
V_s(1) = 0; % Initial velocity

% Simplified equations of motion (static pendulum)
fX_s = @(t, X, V) V;
fV_s = @(t, X, V) (-k*X - c*V - p*g) / (m + p);

% RK4 loop for static pendulum case
for i = 1:length(t_p)-1
    % RK4 for the static pendulum system
    k1X = fX_s(t_p(i), X_s(i), V_s(i));
    k1V = fV_s(t_p(i), X_s(i), V_s(i));
    
    k2X = fX_s(t_p(i)+h/2, X_s(i)+h/2*k1X, V_s(i)+h/2*k1V);
    k2V = fV_s(t_p(i)+h/2, X_s(i)+h/2*k1X, V_s(i)+h/2*k1V);
    
    k3X = fX_s(t_p(i)+h/2, X_s(i)+h/2*k2X, V_s(i)+h/2*k2V);
    k3V = fV_s(t_p(i)+h/2, X_s(i)+h/2*k2X, V_s(i)+h/2*k2V);
    
    k4X = fX_s(t_p(i)+h, X_s(i)+h*k3X, V_s(i)+h*k3V);
    k4V = fV_s(t_p(i)+h, X_s(i)+h*k3X, V_s(i)+h*k3V);
    
    % Summing
    X_s(i+1) = X_s(i) + (h/6)*(k1X + 2*k2X + 2*k3X + k4X);
    V_s(i+1) = V_s(i) + (h/6)*(k1V + 2*k2V + 2*k3V + k4V);
end

% Plot for Pendulum Dynamic
figure;
plot(t_p, X_p, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Displacement: Moving Pendulum');
grid on;

% Plot for Static Pendulum
figure;
plot(t_p, X_s, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Displacement: Static Pendulum');
grid on;

% Combined Plot
figure;
plot(t_p, X_p, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_p, X_s, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Comparison: Moving vs Static Pendulum');
legend('Moving Pendulum', 'Static Pendulum');
grid on;


% Zero-crossing detection function
find_zero_crossings = @(data, time) ...
    time(find(data(1:end-1) .* data(2:end) < 0)); % Time values where displacement changes sign

% Frequency analysis for moving pendulum
zero_crossings_p = find_zero_crossings(X_p, t_p); % Find zero crossings for moving pendulum
periods_p = diff(zero_crossings_p); % Time intervals between zero crossings
freq_p = 1 ./ periods_p; % Frequency calculation

% Frequency analysis for static pendulum
zero_crossings_s = find_zero_crossings(X_s, t_p); % Find zero crossings for static pendulum
periods_s = diff(zero_crossings_s); % Time intervals between zero crossings
freq_s = 1 ./ periods_s; % Frequency calculation

% Midpoints of time intervals for plotting frequency
time_mid_p = zero_crossings_p(1:end-1) + periods_p / 2;
time_mid_s = zero_crossings_s(1:end-1) + periods_s / 2;

% Plot the frequency comparison
figure;
plot(time_mid_p, freq_p, 'b-', 'LineWidth', 1.5);
hold on;
plot(time_mid_s, freq_s, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Frequency (Hz)');
title('Frequency of Oscillation: Moving vs Static Pendulum');
legend('Moving Pendulum', 'Static Pendulum');
grid on;