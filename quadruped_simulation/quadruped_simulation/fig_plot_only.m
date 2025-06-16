function [t,EA,EAd] = fig_plot_only(tout,Xout,Uout,Xdout,Udout,Uext,p)

%% Smooth interpolation
t = (tout(1):p.simTimeStep:tout(end));
X = interp1(tout,Xout,t);
U = interp1(tout,Uout,t);
Xd = interp1(tout,Xdout,t);
Ud = interp1(tout,Udout,t);
Ue = interp1(tout,Uext,t);

%% Precompute EA
nt = length(t);
EA = zeros(nt,3);
EAd = zeros(nt,3);
for ii = 1:nt
    EA(ii,:) = fcn_X2EA(X(ii,:));
    EAd(ii,:) = fcn_X2EA(Xd(ii,:));
end

nt = size(X,1);          % numero di istanti di tempo
RPY = zeros(nt,3);       % preallocazione angoli [roll pitch yaw]

for i = 1:nt
    % Ricostruisci matrice di rotazione 3x3 da vettore riga
    R = reshape(X(i,7:15), [3,3])';  % trasposizione necessaria (righe -> colonne)

    % Calcolo angoli di Eulero da R (sequenza ZYX: yaw-pitch-roll)
    % roll  = rotazione attorno a x
    % pitch = rotazione attorno a y
    % yaw   = rotazione attorno a z
    RPY(i,:) = rotm2eul(R, 'ZYX');
end

%% Zero-order hold for force plotting
t2 = repelem(t,2);
t2(1) = []; t2(end+1) = t2(end);
U2 = repelem(U,2,1);

%% Create figure
figure('Position',[200 100 1000 600]);
set(gcf, 'Color', 'white')
tiledlayout(2,2);

%% Position subplot
nexttile;
plot(t,X(:,1),'r', t,X(:,2),'g', t,X(:,3),'b', ...
     t,Xd(:,1),'r--', t,Xd(:,2),'g--', t,Xd(:,3),'b--','LineWidth',1);
xlabel('Time [s]'); ylabel('Position [m]');
title('COM Position');
legend('x','y','z','x_d','y_d','z_d');

%% Euler Angles (Roll, Pitch, Yaw) subplot
nexttile;
RPY_deg = rad2deg(RPY);  % Converti in gradi
plot(t, RPY_deg(:,1), 'r', ...
     t, RPY_deg(:,2), 'g', ...
     t, RPY_deg(:,3), 'b', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Angle [deg]');
title('Euler Angles: Roll, Pitch, Yaw');
legend('roll','pitch','yaw');


%% angular subplot
% nexttile;
% plot(t2,U2(:,3),'r', t2,U2(:,6),'g', t2,U2(:,9),'b', t2,U2(:,12),'k', ...
%      t,Ud(:,3),'r--', t,Ud(:,6),'g--', t,Ud(:,9),'b--', t,Ud(:,12),'k--','LineWidth',1);
% xlabel('Time [s]'); ylabel('Force Fz [N]');
% title('RPY');
% legend('Fz_1','Fz_2','Fz_3','Fz_4','Fz_{1,d}','Fz_{2,d}','Fz_{3,d}','Fz_{4,d}');

%% Velocity subplot
nexttile;
plot(t,X(:,4),'r', t,X(:,5),'g', t,X(:,6),'b', ...
     t,Xd(:,4),'r--', t,Xd(:,5),'g--', t,Xd(:,6),'b--','LineWidth',1);
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('COM Velocity');
legend('v_x','v_y','v_z','v_{x,d}','v_{y,d}','v_{z,d}');

%% Angular velocity subplot
nexttile;
plot(t,X(:,16),'r', t,X(:,17),'g', t,X(:,18),'b', ...
     t,Xd(:,16),'r--', t,Xd(:,17),'g--', t,Xd(:,18),'b--','LineWidth',1);
xlabel('Time [s]'); ylabel('Angular Vel [rad/s]');
title('Angular Velocity');
legend('\omega_x','\omega_y','\omega_z','\omega_{x,d}','\omega_{y,d}','\omega_{z,d}');

% %% Control force Fz subplot
% nexttile;
% plot(t2,U2(:,3),'r', t2,U2(:,6),'g', t2,U2(:,9),'b', t2,U2(:,12),'k', ...
%      t,Ud(:,3),'r--', t,Ud(:,6),'g--', t,Ud(:,9),'b--', t,Ud(:,12),'k--','LineWidth',1);
% xlabel('Time [s]'); ylabel('Force Fz [N]');
% title('GRFs Z-axis');
% legend('Fz_1','Fz_2','Fz_3','Fz_4','Fz_{1,d}','Fz_{2,d}','Fz_{3,d}','Fz_{4,d}');

%% ======= INFO TEXT SOTTO I GRAFICI =======
gait_labels = {'trot', 'bound', 'pacing', 'gallop', 'trot run', 'crawl'};

% Nome andatura
if p.gait >= 0 && p.gait <= 5
    gait_str = gait_labels{p.gait + 1}; % indice MATLAB da 1
else
    gait_str = 'unknown';
end

% Accelerazione desiderata massima (norma del vettore acc)
acc_max_des = max(vecnorm(Xd(:,10:12), 2, 2));

% Errore di posizione lungo x
error_position_along_x = Xd(:,1) - X(:,1);

%% ======= INFO TEXT PER COMMAND WINDOW =======
gait_labels = {'trot', 'bound', 'pacing', 'gallop', 'trot run', 'crawl'};

% Nome andatura
if p.gait >= 0 && p.gait <= 5
    gait_str = gait_labels{p.gait + 1}; % MATLAB: 1-based indexing
else
    gait_str = 'unknown';
end

% Accelerazione desiderata massima
acc_max_des = max(vecnorm(Xd(:,10:12), 2, 2));

% Errore massimo in posizione lungo x
error_position_along_x = Xd(:,1) - X(:,1);
max_error_x = max(error_position_along_x);

% ======= ERRORI VELOCITÀ ==========
err_vx = abs(Xd(:,4) - X(:,4));
err_vy = abs(Xd(:,5) - X(:,5));
err_vz = abs(Xd(:,6) - X(:,6));

% ======= ERRORI VELOCITÀ ANGOLARE ==========
err_wx = abs(Xd(:,16) - X(:,16));
err_wy = abs(Xd(:,17) - X(:,17));
err_wz = abs(Xd(:,18) - X(:,18));


% Stampa a video (Command Window)
fprintf('\n===== INFORMAZIONI DI SIMULAZIONE =====\n');
fprintf('Velocità desiderata max       : %.2f m/s\n', max(Xd(:,4)));
fprintf('Massa robot                   : %.2f kg\n', p.mass);
fprintf('Accelerazione desiderata max : %.2f m/s²\n', acc_max_des);
fprintf('Andatura                     : %d (%s)\n', p.gait, gait_str);
fprintf('Errore max posizione su x    : %.2f m\n', max_error_x);
fprintf('=======================================\n\n');

% ======= STAMPA SU COMMAND WINDOW =======
fprintf('\n===== ERRORI VELOCITÀ MASSIMI =====\n');
fprintf('Ampiezza Errore max velocità su x     : %.3f m/s\n', abs(max(err_vx(end/2:end))));
fprintf('Ampiezza Errore max velocità su y     : %.3f m/s\n', abs(max(err_vy(end/2:end))));
fprintf('Ampiezza Errore max velocità su z     : %.3f m/s\n', abs(max(err_vz(end/2:end))));

fprintf('\n===== ERRORI VEL. ANGOLARE MASSIMI =====\n');
fprintf('Ampiezza Errore max ω_x               : %.3f rad/s\n', abs(max(err_wx(end/2:end))));
fprintf('Ampiezza Errore max ω_y               : %.3f rad/s\n', abs(max(err_wy(end/2:end))));
fprintf('Ampiezza Errore max ω_z               : %.3f rad/s\n', abs(max(err_wz(end/2:end))));

fprintf('\n===== GRFs MASSIME =====\n');
fprintf('GRF max (reale)    : %.2f N\n', max([max(U(:,3)),max(U(:,6)),max(U(:,9)),max(U(:,12))]));

    
