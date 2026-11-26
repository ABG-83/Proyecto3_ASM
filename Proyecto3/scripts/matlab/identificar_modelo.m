%% IDENTIFICAR_MODELO  Modelo de la planta a partir de ensayos a lazo abierto
% Responsable: Integrante B  (enunciado, seccion 2.3)
%
% Entradas: CSV en datos/identificacion/ (varios u, dos velocidades iniciales).
% Salidas:  K, tau, L (retardo del servo), zona muerta, saturacion,
%           G(s) local con sus condiciones de validez, polos y ceros.
% Punto de partida: Tarea2/analizar_escalon.m
clear; clc;

% TODO (B): listar los archivos de datos/identificacion/
% TODO (B): ajustar el modelo (por ejemplo, primer orden con retardo)
% TODO (B): guardar el modelo para C y D:
% save('modelo_planta.mat', 'G', 'K', 'tau', 'L', 'condiciones');
