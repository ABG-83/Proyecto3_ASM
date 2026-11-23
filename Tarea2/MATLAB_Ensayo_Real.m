%% Banco de frenado: ensayo REAL, ultimo OA completo recibido en el chat
% Ejecutar este archivo completo en MATLAB (boton Run/Ejecutar).
% No conecta ni mueve el ESP32. No necesita repetir el ensayo fisico.
% MATLAB base: graficas medidas. Control System Toolbox: tf/step/impulse.
% Fuente: registro del usuario, ACTIVE6006ms, RELEASE12006ms, final13926ms.
% Se extraen filas originales: NO se fabrican muestras antes/despues.
% Una marca por vuelta. Cada velocidad es un PROMEDIO entre dos flancos.
% No borrar otras variables ni cerrar figuras de trabajos anteriores.
clc;

% [t_ms, periodo_us, edad_ms, pulsos, u, fase]
% fase0=WARM; fase1=ACTIVE; fase2=RELEASE. Todos estos registros pwm160.
datos = [
4306 583000 99 11 0 0
4806 562000 37 12 0 0
5406 556000 81 13 0 0
5906 557000 24 14 0 0
6526 617000 27 15 1 1
7226 626000 101 16 1 1
7826 619000 82 17 1 1
8426 614000 68 18 1 1
9026 614000 54 19 1 1
9626 611000 43 20 1 1
10226 611000 32 21 1 1
10826 611000 21 22 1 1
11426 608000 13 23 1 1
12026 607000 6 24 0 2
12626 574000 32 25 0 2
13226 565000 67 26 0 2
13826 563000 104 27 0 2
];
PPR=1; tAplicar=6.006; tLiberar=12.006; tFinRegistro=13.926;
periodo=datos(:,2)/1e6;
tFlanco=(datos(:,1)-datos(:,3))/1000;
tInicio=tFlanco-periodo;
tMedio=tFlanco-periodo/2;
rpm=60./(PPR*periodo); w=rpm*2*pi/60;
assert(all(diff(datos(:,4))==1),'Faltan pulsos en la seleccion');
assert(all(abs(diff(tFlanco)-periodo(2:end))<.003),...
    'Intervalos incompatibles con los flancos transcritos');

% Ventana anterior estable: periodos COMPLETOS dentro de los ultimos2s.
libre=datos(:,6)==0 & tInicio>=tAplicar-2;
frenado=datos(:,6)==1;
assert(nnz(libre)>=3 && nnz(frenado)>=6,'Datos insuficientes');
rpmLibre=median(rpm(libre)); wLibre=rpmLibre*2*pi/60;
idx=find(frenado); rpmFinal=median(rpm(idx(end-2:end)));
rpmMin=min(rpm(frenado));
K_observada=(rpmFinal-rpmLibre)*2*pi/60; % u0->1, [rad/s]/u
fprintf('DATOS DEL ULTIMO ENSAYO\n');
fprintf('Freno:1200->1350us; motorPWM160 constante; escalon0->1.\n');
fprintf('Libre (mediana3periodos):%.2fRPM\n',rpmLibre);
fprintf('Minimo medido por periodo:%.2fRPM\n',rpmMin);
fprintf('Mediana3ultimos periodos frenados:%.2fRPM\n',rpmFinal);
fprintf('Cambio final aproximado:%.2fRPM (%.2f%%)\n',rpmFinal-rpmLibre,100*(rpmFinal/rpmLibre-1));
fprintf('Ganancia incremental aproximada:%.5f(rad/s)/u\n',K_observada);

fig1=figure('Name','Ensayo fisico: entrada y velocidad','Color','w');
subplot(2,1,1);
stairs([tAplicar-2 tAplicar tLiberar tFinRegistro],[0 1 0 0],...
    'LineWidth',1.8);ylim([-.1 1.15]);grid on;
ylabel('Orden de freno u');title('Escalon aplicado; PWM del motor =160');
subplot(2,1,2);
plot(tMedio,rpm,'o-','LineWidth',1.2,'MarkerFaceColor',[.1 .4 .75]);hold on;
xline(tAplicar,'--','Aplicar');xline(tLiberar,'--','Liberar');
yline(rpmLibre,':','Referencia libre');grid on;
xlim([tAplicar-2 tFinRegistro]);xlabel('Tiempo del ensayo [s]');ylabel('RPM');
title('Promedios entre flancos, situados en el centro de cada intervalo');

%% Modelo empirico discreto de la RESPUESTA MEDIDA
% Se remuestrea a0.6s, comparable al intervalo real entre pulsos.
% La interpolacion NO agrega informacion ni mediciones independientes.
% Se usa FIR porque la respuesta cae y luego recupera parcialmente: un
% primer orden monotono no describe esa forma. No se inventa una tau.
% Modelo DESCRIPTIVO local: no validado para otras presiones/velocidades.
Ts=.6; % Paso del modelo; NO cambia los20ms nominales del firmware.
ti=tMedio(frenado)-tAplicar;
yi=w(frenado)-wLibre;
tModelo=(0:Ts:floor(max(ti)/Ts)*Ts)';
assert(numel(tModelo)>=6,'Pocos puntos para modelo discreto');
respuestaEscalon=[0;interp1(ti,yi,tModelo(2:end),'linear')];
assert(all(isfinite(respuestaEscalon)),'No extrapolar datos ausentes');
% Para escalon unitario: s[k]=sum(b[0:k]); por tanto b[k]=s[k]-s[k-1].
b=[respuestaEscalon(1);diff(respuestaEscalon)]';
den=[1 zeros(1,numel(b)-1)]; % denominador z^N; FIR causal, polos en0.
resultado=struct('fuente','Ultimo OA completo recibido por chat',...
    'tipo','FIR descriptivo de respuesta medida','Ts_s',Ts,'b',b,'den',den,...
    'rpm_libre',rpmLibre,'rpm_minima',rpmMin,'rpm_final_aprox',rpmFinal,...
    'ganancia_observada_rad_s_por_u',K_observada,...
    'validado_en_otro_ensayo',false);
carpeta=fileparts(mfilename('fullpath'));
save(fullfile(carpeta,'resultado_ensayo_real.mat'),'resultado','datos');
saveas(fig1,fullfile(carpeta,'01_Ensayo_medido.png'));

if exist('tf','file') && exist('step','file') && exist('impulse','file')
    G=tf(b,den,Ts);G.InputName='Orden incremental del freno';
    G.OutputName='Cambio de velocidad angular medida';
    disp('Modelo discreto FIR aproximado:');disp(G);
    [s,tm]=step(G,tModelo);s=squeeze(s);s=s(:);
    assert(max(abs(s-respuestaEscalon))<1e-7,...
        'La conversion a tf no conserva la respuesta al escalon');
    fig2=figure('Name','Modelo y datos del escalon','Color','w');
    plot(ti,w(frenado)*60/(2*pi),'o','MarkerSize',7);hold on;
    stairs(tm,(wLibre+s)*60/(2*pi),'LineWidth',1.5);grid on;
    xlabel('Tiempo desde aplicar el freno [s]');ylabel('RPM');
    legend('Promedio real entre flancos','Modelo discreto descriptivo','Location','best');
    title('Aproximacion a0.6s: no identifica dinamicas mas rapidas');
    fig3=figure('Name','Respuesta al impulso en MATLAB','Color','w');
    impulse(G,0:Ts:6);grid on;
    title('Respuesta al impulso DEL MODELO empirico discreto');
    saveas(fig2,fullfile(carpeta,'02_Modelo_escalon.png'));
    saveas(fig3,fullfile(carpeta,'03_Impulso_MATLAB.png'));
    save(fullfile(carpeta,'resultado_ensayo_real.mat'),'resultado','datos','G');
    fprintf('Graficas y MAT guardados en:%s\n',carpeta);
    fprintf('El impulso es simulado por MATLAB; no fue aplicado fisicamente.\n');
else
    warning(['Falta Control System Toolbox (tf/step/impulse). La grafica de datos ',...
        'esta lista; la respuesta al impulso en MATLAB sigue pendiente.']);
end
fprintf(['LIMITES:1marca/vuelta, observaciones cada~0.6s, calibracion local, ',...
    'sin medicion de torque/fuerza. No se afirma PID sintonizado.\n']);
