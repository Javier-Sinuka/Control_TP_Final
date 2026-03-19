% Simulacion del secadero de yerba mate
% Compara:
% 1) Planta sin mejoras (lazo abierto con ajuste nominal de potencia)
% 2) Lazo cerrado con controlador PI lineal
% 3) Lazo cerrado con PI + no linealidades (saturacion, dinamica de actuador,
%    dinamica de sensor y variacion paramétrica por cambio de carga)
%
% El script genera metricas de desempeno y figuras comparativas.

clear; clc; close all;

% Parametros nominales
Kp = 0.02;       % degC/W
tau_p = 200;     % s
tau_a = 5;       % s
tau_s = 1;       % s

% Controlador PI seleccionado en el informe
Kc = 150;
Ti = 200;

% Escenario de simulacion
dt = 0.5;
t_end = 1800;
t = 0:dt:t_end;
n = length(t);

% Referencia: escalon de 20 degC a partir de t = 10 s
r = zeros(1, n);
r(t >= 10) = 20;

d_nom = zeros(1, n);
d_dist = zeros(1, n);
d_dist(t >= 600) = -6;

% Potencia nominal en lazo abierto para alcanzar 20 degC en condiciones ideales
Q_nom = 20 / Kp;

% Simulaciones nominales
[T_open_nom, Q_open_nom] = simular_abierto(t, r, d_nom, Kp, tau_p, Q_nom, dt);
[T_lin_nom, Q_lin_nom] = simular_pi_lineal(t, r, d_nom, Kp, tau_p, Kc, Ti, dt);
[T_nl_nom, Tm_nl_nom, Qcmd_nl_nom, Qact_nl_nom] = simular_pi_nolineal(t, r, d_nom, Kc, Ti, tau_a, tau_s, dt, false);

% Simulaciones con perturbaciones / no linealidades
[T_open_dist, Q_open_dist] = simular_abierto(t, r, d_dist, Kp, tau_p, Q_nom, dt);
[T_lin_dist, Q_lin_dist] = simular_pi_lineal(t, r, d_dist, Kp, tau_p, Kc, Ti, dt);
[T_nl_dist, Tm_nl_dist, Qcmd_nl_dist, Qact_nl_dist] = simular_pi_nolineal(t, r, d_dist, Kc, Ti, tau_a, tau_s, dt, true);

% Metricas nominales de cumplimiento de especificaciones
metric_open_nom = calcular_metricas(t, r, T_open_nom, 10);
metric_lin_nom = calcular_metricas(t, r, T_lin_nom, 10);
metric_nl_nom = calcular_metricas(t, r, T_nl_nom, 10);

% Error residual ante perturbacion permanente
err_open_dist = r(end) - mean(T_open_dist(end-20:end));
err_lin_dist = r(end) - mean(T_lin_dist(end-20:end));
err_nl_dist = r(end) - mean(T_nl_dist(end-20:end));

fprintf('\nRESULTADOS NOMINALES\n');
fprintf('Planta sin mejoras: ess = %.3f degC, Mp = %.3f %% , ts = %.1f s\n', ...
  metric_open_nom.ess, metric_open_nom.Mp, metric_open_nom.ts);
fprintf('PI lineal:          ess = %.3f degC, Mp = %.3f %% , ts = %.1f s\n', ...
  metric_lin_nom.ess, metric_lin_nom.Mp, metric_lin_nom.ts);
fprintf('PI no lineal:       ess = %.3f degC, Mp = %.3f %% , ts = %.1f s\n', ...
  metric_nl_nom.ess, metric_nl_nom.Mp, metric_nl_nom.ts);

fprintf('\nERROR EN REGIMEN CON PERTURBACION PERMANENTE\n');
fprintf('Sin mejoras: %.3f degC\n', err_open_dist);
fprintf('PI lineal:   %.3f degC\n', err_lin_dist);
fprintf('PI no lineal %.3f degC\n', err_nl_dist);

% Graficos
figure(1);
plot(t, r, 'k--', 'LineWidth', 1.6); hold on;
plot(t, T_open_nom, 'LineWidth', 1.8);
plot(t, T_lin_nom, 'LineWidth', 1.8);
plot(t, T_nl_nom, 'LineWidth', 1.8);
grid on;
xlabel('Tiempo [s]');
ylabel('Temperatura [^oC]');
title('Respuesta nominal al escalon');
legend('Referencia', 'Sin mejoras', 'PI lineal', 'PI con no linealidades', 'Location', 'southeast');

figure(2);
plot(t, r, 'k--', 'LineWidth', 1.6); hold on;
plot(t, T_open_dist, 'LineWidth', 1.8);
plot(t, T_lin_dist, 'LineWidth', 1.8);
plot(t, T_nl_dist, 'LineWidth', 1.8);
grid on;
xlabel('Tiempo [s]');
ylabel('Temperatura [^oC]');
title('Respuesta con perturbacion y no linealidades');
legend('Referencia', 'Sin mejoras', 'PI lineal', 'PI con no linealidades', 'Location', 'southeast');

figure(3);
plot(t, Q_open_dist, 'LineWidth', 1.8); hold on;
plot(t, Q_lin_dist, 'LineWidth', 1.8);
plot(t, Qcmd_nl_dist, 'LineWidth', 1.8);
plot(t, Qact_nl_dist, 'LineWidth', 1.8);
grid on;
xlabel('Tiempo [s]');
ylabel('Potencia / accion de control [W]');
title('Accion de control comparativa');
legend('Sin mejoras', 'PI lineal', 'PI no lineal - mando', 'PI no lineal - actuador', 'Location', 'southeast');

function out = calcular_metricas(t, r, y, t_step)
  idx = find(t >= t_step, 1);
  r_final = r(end);
  yss = mean(y(end-20:end));
  out.ess = r_final - yss;
  if abs(r_final) > 1e-9
    out.Mp = max(0, (max(y(idx:end)) - r_final) / abs(r_final) * 100);
  else
    out.Mp = 0;
  end

  banda = 0.02 * abs(r_final);
  ts = NaN;
  for k = idx:length(t)
    if all(abs(y(k:end) - r_final) <= banda)
      ts = t(k) - t_step;
      break;
    end
  end
  out.ts = ts;
end

function [T, Q] = simular_abierto(t, r, d, Kp, tau_p, Q_nom, dt)
  n = length(t);
  T = zeros(1, n);
  Q = zeros(1, n);
  for k = 1:n-1
    if t(k) >= 10
      Q(k) = Q_nom;
    end
    dT = (-T(k) + Kp * Q(k) + d(k)) / tau_p;
    T(k+1) = T(k) + dt * dT;
  end
  Q(n) = Q(n-1);
end

function [T, Q] = simular_pi_lineal(t, r, d, Kp, tau_p, Kc, Ti, dt)
  n = length(t);
  T = zeros(1, n);
  Q = zeros(1, n);
  I = 0;
  for k = 1:n-1
    e = r(k) - T(k);
    I = I + e * dt;
    Q(k) = Kc * (e + (1 / Ti) * I);
    dT = (-T(k) + Kp * Q(k) + d(k)) / tau_p;
    T(k+1) = T(k) + dt * dT;
  end
  Q(n) = Q(n-1);
end

function [T, Tm, Qcmd, Qact] = simular_pi_nolineal(t, r, d, Kc, Ti, tau_a, tau_s, dt, con_cambio_carga)
  n = length(t);
  T = zeros(1, n);
  Tm = zeros(1, n);
  Qcmd = zeros(1, n);
  Qact = zeros(1, n);
  I = 0;
  for k = 1:n-1
    if con_cambio_carga && t(k) >= 600
      Kp_eff = 0.019;
      tau_eff = 230;
    else
      Kp_eff = 0.02;
      tau_eff = 200;
    end
    y_sensor = Tm(k) + 0.0003 * Tm(k)^2;
    e = r(k) - y_sensor;
    u_unsat = Kc * (e + (1 / Ti) * I);
    u_sat = min(max(u_unsat, 0), 3000);
    if abs(u_unsat - u_sat) < 1e-9
      I = I + e * dt;
    end
    Qcmd(k) = u_sat;
    dQ = (-Qact(k) + Qcmd(k)) / tau_a;
    Qact(k+1) = Qact(k) + dt * dQ;
    dT = (-T(k) + Kp_eff * Qact(k) + d(k)) / tau_eff;
    T(k+1) = T(k) + dt * dT;
    dTm = (-Tm(k) + T(k)) / tau_s;
    Tm(k+1) = Tm(k) + dt * dTm;
  end
  Qcmd(n) = Qcmd(n-1);
end
