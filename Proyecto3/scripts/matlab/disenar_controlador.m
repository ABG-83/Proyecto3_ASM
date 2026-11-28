%% DISENAR_CONTROLADOR  Diseno del PID digital del banco de frenado
% Responsable: Integrante C  (enunciado, seccion 2.4)
%
% Pasos:
%   1. Modelo de la planta (de Integrante B; si no existe, el de la Tarea 2).
%   2. Sintonia inicial por SIMC (Skogestad, 2003): PI y PID.
%   3. Discretizacion: planta con ZOH, controlador con Euler hacia atras
%      (igual que Control.h).
%   4. Polos y ceros del controlador y del lazo cerrado; margenes.
%   5. Simulacion no lineal de la rampa con saturacion y anti-windup.
%
% Requiere Control System Toolbox.
clear; clc; close all;

%% 1. Modelo de la planta ----------------------------------------------------
% G(s) = dW(s)/dU(s) = -K e^(-L s) / ((tau s + 1)(tau_a s + 1))
if isfile('modelo_planta.mat')
    load('modelo_planta.mat', 'K', 'tau', 'tau_a', 'L');
    fuente = 'modelo identificado (Integrante B)';
else
    K = 29.17; tau = 5; tau_a = 0.12; L = 0.15;    % Tarea 2, Pregunta 5
    fuente = 'modelo supuesto de la Tarea 2';
end
fprintf('Planta: %s\n  K = %.2f rad/s, tau = %.2f s, tau_a = %.2f s, L = %.2f s\n', ...
        fuente, K, tau, tau_a, L);

Ts = 0.020;                        % periodo de muestreo [s] (Tarea 2, Pregunta 2)

s = tf('s');
G = -K / ((tau*s + 1)*(tau_a*s + 1));
G.InputDelay = L;

% El freno es de accion inversa: e = w - r y u = C*e. Con esa convencion,
% el lazo equivale a un lazo normal (e = r - w) con la planta -G, que tiene
% ganancia positiva. Todo el analisis se hace con Gp = -G.
Gp = -G;

%% 2. Sintonia SIMC ----------------------------------------------------------
% PI: regla de la mitad -> primer orden con retardo
tau1  = tau + tau_a/2;             % constante de tiempo equivalente
theta = L   + tau_a/2;             % retardo equivalente
tau_c = theta;                     % recomendacion SIMC: tau_c = theta
Kp_PI = tau1 / (K*(tau_c + theta));
Ti_PI = min(tau1, 4*(tau_c + theta));
Ki_PI = Kp_PI / Ti_PI;

% PID: segundo orden con retardo (forma serie) -> forma paralela
tau_c2 = L;
Kc  = tau / (K*(tau_c2 + L));
tI  = min(tau, 4*(tau_c2 + L));
tD  = tau_a;
Kp_PID = Kc*(1 + tD/tI);
Ti_PID = tI + tD;
Td_PID = tI*tD/(tI + tD);
Ki_PID = Kp_PID / Ti_PID;
Kd_PID = Kp_PID * Td_PID;

fprintf('\nSintonia SIMC\n');
fprintf('  PI : Kp = %.3f, Ki = %.3f 1/s (Ti = %.2f s)\n', Kp_PI, Ki_PI, Ti_PI);
fprintf('  PID: Kp = %.3f, Ki = %.3f 1/s, Kd = %.4f s\n', Kp_PID, Ki_PID, Kd_PID);

%% 3. Discretizacion ---------------------------------------------------------
Gz = c2d(Gp, Ts, 'zoh');           % la planta "ve" un retenedor de orden cero

Tf   = 0.10;                       % filtro derivativo [s] (igual que Control.h)
alfa = Tf/(Tf + Ts);
z = tf('z', Ts);

% Igual que Control.h:  P = Kp*e,  I = Ki*Ts*sum(e),
%                       D = Kd*(1-alfa)*(z-1)/(Ts*(z-alfa))*e   (linealizado)
Cz = @(kp, ki, kd) kp + ki*Ts*z/(z - 1) + kd*(1 - alfa)*(z - 1)/(Ts*(z - alfa));

Cz_PI  = minreal(Cz(Kp_PI,  Ki_PI,  0));
Cz_PID = minreal(Cz(Kp_PID, Ki_PID, Kd_PID));

%% 4. Polos, ceros y margenes ------------------------------------------------
casos = {'PI', Cz_PI; 'PID', Cz_PID};
for i = 1:2
    nombre = casos{i,1};  C = casos{i,2};
    Lz = C*Gz;                      % lazo abierto
    T  = feedback(Lz, 1);           % lazo cerrado r -> w
    [Gm, Pm, ~, Wc] = margin(Lz);
    pT = pole(T);
    fprintf('\n%s\n', nombre);
    fprintf('  Polos del controlador: %s\n', mat2str(pole(C).', 4));
    fprintf('  Ceros del controlador: %s\n', mat2str(zero(C).', 4));
    fprintf('  |polo| maximo lazo cerrado: %.4f -> %s\n', max(abs(pT)), ...
            ternario(all(abs(pT) < 1), 'estable', 'INESTABLE'));
    fprintf('  Margen de ganancia: %.2f (%.1f dB), margen de fase: %.1f grados, wc = %.2f rad/s\n', ...
            Gm, 20*log10(Gm), Pm, Wc);
    figure('Name', ['Polos y ceros - ' nombre]);
    pzmap(T); zgrid; title(['Lazo cerrado en Z (' nombre ')']);
end

%% 5. Simulacion no lineal de la rampa ---------------------------------------
% Planta fisica de la Pregunta 5 (friccion seca y parada en cero), con el
% mismo PID del firmware (saturacion + anti-windup). Se compara PI contra PID.
J = 0.006; b = 0.0012; tau_cou = 0.002; kb = 0.035;   % Tarea 2, Pregunta 5
w0 = 120*2*pi/60;                  % 120 rpm en rad/s
a  = 4;                            % pendiente de la rampa [rad/s^2]
fprintf('\nRampa: w0 = %.2f rad/s, a = %.1f rad/s^2\n', w0, a);
fprintf('  Desaceleracion libre ~ %.2f rad/s^2, con freno a fondo ~ %.2f rad/s^2\n', ...
        (b*w0 + tau_cou)/J, (b*w0 + tau_cou + kb)/J);

ganancias = [Kp_PI Ki_PI 0; Kp_PID Ki_PID Kd_PID];
figure('Name', 'Seguimiento de la rampa');
for i = 1:2
    [t, r, w, u] = simular(ganancias(i,:), Ts, Tf, w0, a, J, b, tau_cou, kb, tau_a, L);
    activo = r > 0;
    rmse = sqrt(mean((w(activo) - r(activo)).^2));
    fprintf('  %s: RMSE = %.3f rad/s, saturacion = %.1f %% del tiempo\n', ...
            casos{i,1}, rmse, 100*mean(u(activo) >= 1));
    subplot(2,1,1); hold on; plot(t, w, 'DisplayName', ['w ' casos{i,1}]);
    subplot(2,1,2); hold on; stairs(t, u, 'DisplayName', ['u ' casos{i,1}]);
end
subplot(2,1,1); plot(t, r, 'k--', 'DisplayName', 'referencia');
ylabel('\omega [rad/s]'); legend; grid on;
subplot(2,1,2); ylabel('u'); xlabel('t [s]'); legend; grid on; ylim([-0.05 1.05]);

%% Funciones locales ---------------------------------------------------------
function [t, r, w, u] = simular(g, Ts, Tf, w0, a, J, b, tc, kb, tau_a, L)
    dt = 1e-3; tfin = 6; n = round(tfin/dt); paso = round(Ts/dt);
    nd = round(L/dt); cola = zeros(1, nd);         % retardo del servo
    wk = w0; v = 0; I = 0; df = 0; wAnt = w0; uk = 0;
    alfa = Tf/(Tf + Ts);
    t = (0:paso:n-1)*dt; m = numel(t);
    r = zeros(1,m); w = zeros(1,m); u = zeros(1,m); j = 0;
    for k = 0:n-1
        if mod(k, paso) == 0                       % ---- PID, igual que Control.h
            j = j + 1; tk = k*dt;
            rk = max(w0 - a*tk, 0);
            e  = wk - rk;
            df = alfa*df + (1 - alfa)*(wk - wAnt)/Ts;  wAnt = wk;
            us = g(1)*e + I + g(3)*df;
            uk = min(max(us, 0), 1);
            if ~((us > 1 && e > 0) || (us < 0 && e < 0))
                I = min(max(I + g(2)*Ts*e, 0), 1);
            end
            r(j) = rk; w(j) = wk; u(j) = uk;
        end
        cola = [cola(2:end) uk];  ud = cola(1);   % ---- planta fisica
        v = v + dt*(ud - v)/tau_a;
        if wk > 0
            wk = max(wk + dt*(-b*wk - tc - kb*v)/J, 0);
        end
    end
end

function s = ternario(c, a, b)
    if c, s = a; else, s = b; end
end