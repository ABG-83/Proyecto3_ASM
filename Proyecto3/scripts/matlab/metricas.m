function M = metricas(T)
% METRICAS  Indicadores de desempeno de un ensayo
% Responsable: Integrante D  (enunciado, seccion 2.6)
%
% Entrada: tabla de leer_ensayo.
% Salida:  struct con IAE, RMSE, tiempo de frenado, % de tiempo saturado
%          y numero de oscilaciones.

% TODO (D)
M = struct('IAE', NaN, 'RMSE', NaN, 't_frenado', NaN, ...
           'saturacion', NaN, 'oscilaciones', NaN);
end
