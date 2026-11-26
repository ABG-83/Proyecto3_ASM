%% DISENAR_CONTROLADOR  Diseno del PID con el modelo identificado
% Responsable: Integrante C  (enunciado, seccion 2.4)
%
% Entradas: modelo_planta.mat (de Integrante B).
% Salidas:  ganancias iniciales, C(s) y C(z) con la discretizacion usada,
%           polos y ceros del controlador y del lazo cerrado (Laplace y Z),
%           respuesta simulada a la rampa decreciente con saturacion.
% Extra: comparar dos periodos de muestreo (por ejemplo, 20 ms y 60 ms).
clear; clc;

% TODO (C): load('modelo_planta.mat')
% TODO (C): sintonia inicial (por ejemplo, SIMC como en la Tarea 2)
% TODO (C): c2d, pzmap del controlador y del lazo cerrado
