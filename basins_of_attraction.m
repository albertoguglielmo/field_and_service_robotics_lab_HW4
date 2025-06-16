%% 

clc
close all
clear all

% Parameters
g = 9.81;          % gravity (m/s^2)
l = 1.0;           % leg length (m) %%MODIFY HERE%% 
alpha = pi/8;      % half inter-leg angle (rad) %%MODIFY HERE%% 
gamma = 0.08;      % slope angle (rad) %%MODIFY HERE%% 

% Intervallo condizioni iniziali
theta_range = -0.31:0.01:0.47;
thetadot_range = linspace(-2, 2, 101);
[X, Y] = meshgrid(theta_range, thetadot_range);
map = zeros(size(X)); 

% Parametri attrattori
 theta1 = -0.3126;
 theta2 = 0.4726;


tol_theta = 0.05;
tol_thetadot = 0.02;
tol_cycle = 0.01;
N_check = 5;  % numero minimo di impatti da controllare
total = numel(X);
impact_matrix = zeros(size(X));  % Nuova matrice per numero di impatti

% Loop su ogni condizione iniziale
for i = 1:total
    impact_count = 0;  % Conta gli impatti per questa condizione iniziale
    if mod(i, 500) == 0 || i == 1 || i == total
        fprintf('Avanzamento: %.1f%%\n', 100 * i / total);
    end

    y0 = [X(i); Y(i)];
    t0 = 0; tf = 10; dt = 0.01;
    double_support = 0;
    impact_states = [];

    for step = 1:30
        options = odeset('Events', @(t, y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
        [~, ~, te, ye, ~] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);

        if isempty(te)
            break; % nessun impatto
        end

        [y0, double_support] = impact_map(ye, alpha, g, l);
        impact_count = impact_count + 1;  % Aumenta il conteggio impatti

        % Salva lo stato per verifica ciclo limite
        impact_states = [impact_states, y0];
        if size(impact_states, 2) > N_check
            impact_states = impact_states(:, end-N_check+1:end);  
        end

        t0 = te;

        if double_support
            break;
        end
    end
    impact_matrix(i) = impact_count;  % Salva il numero di impatti nella matrice
    % === Classificazione finale ===
    if double_support && abs(y0(1) - theta1) < tol_theta && abs(y0(2)) < tol_thetadot 
            map(i) = 1; 
    elseif double_support && abs(y0(1) - theta2) < tol_theta  && abs(y0(2)) < tol_thetadot
            map(i) = 2; 
    elseif abs(y0(2))==0 && abs(y0(1))==0
            map(i) = 4;  
    else
            map(i) = 3;
    end
end

figure;
hold on;
colors = [0 0.5 0; 0 0 1; 1 0 0; 0.7 0.7 0];
labels = {'Equilibrio -0.3126', 'Equilibrio 0.4726', 'Ciclo limite', 'Altro'};

for b = 1:4
    idx = map == b;
    scatter(X(idx), Y(idx), 36, colors(b,:), 'filled');
end

legend(labels, 'Location', 'best');
xlabel('\theta (rad)');
ylabel('\theta dot (rad/s)');
title('basins of attraction');
grid on;

exportgraphics(gcf, 'Hw4_es4_basin.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');

function dydt = dynamics(~, y, g, l, ds)
    theta = y(1);
    thetadot = y(2);
    if (~ds)
        dtheta = thetadot;
        dthetadot = (g/l) * sin(theta);
    else
        dtheta = 0;
        dthetadot = 0;
    end
    dydt = [dtheta; dthetadot];
end

function [value, isterminal, direction] = impact_event(~, y, alpha,gamma)
    
    value = [y(1)-alpha-gamma; y(1)-gamma+alpha];% Trigger when theta = gamma+alpha
                                     %Trigger when theta = gamma-alpha
    isterminal = [1;1];         % Stop the integration
    direction = [1;-1];          % Detect only when increasing
end

function [yplus,ds] = impact_map(y_minus, alpha,g,l)%minus: before impact time; plus: after impact time
    if (y_minus(2)>=0)
        theta_plus = y_minus(1)-2*alpha;
    else
        theta_plus = y_minus(1)+2*alpha;
    end
    thetadot_plus = cos(2*alpha) * y_minus(2);
    if (thetadot_plus < 0.01*sqrt(g/l) && thetadot_plus >-0.01*sqrt(g/l)) 
        thetadot_plus = 0;
        ds = 1;
    else
        ds = 0;
    end
    yplus = [theta_plus; thetadot_plus];
end