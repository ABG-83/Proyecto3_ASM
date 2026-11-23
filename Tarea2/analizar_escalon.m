function resultado = analizar_escalon(archivo,ppr)
% OA real -> modelo incremental K/(tau*s+1) con retardo -> impulso.
% Ajusta PROMEDIOS entre flancos: no trata filas retenidas como muestras nuevas.
% Uso: resultado=analizar_escalon('ensayo_OA.csv',1)
% Requiere MATLAB. tf/impulse requieren Control System Toolbox.
if nargin<1
    [f,p]=uigetfile('*.csv'); if isequal(f,0),resultado=[];return;end
    archivo=fullfile(p,f);
end
if nargin<2,ppr=1;end
validateattributes(ppr,{'numeric'},{'scalar','integer','>=',1,'<=',20});
opts=detectImportOptions(archivo,'CommentStyle','#');
opts=setvartype(opts,{'estado','modo'},'string');T=readtable(archivo,opts);
required={'t_ms','w','u','pulsos','periodo_us','nuevo','edad_ms','valido',...
    'cota','pwm','estado','modo','huecos','log_perdido'};
assert(all(ismember(required,T.Properties.VariableNames)),'CSV incompatible');
T=T(string(T.modo)=="OA",:);assert(height(T)>0,'No hay ensayo OA');
% Selecciona el ultimo ensayo OA cuando se capturaron varios.
resets=find(diff(T.t_ms)<0);if ~isempty(resets),T=T(resets(end)+1:end,:);end
active=string(T.estado)=="ACTIVE";
assert(nnz(active)>=10,'Faltan filas ACTIVE de un escalon OA completo');
assert(any(string(T.estado)=="RELEASE"),'Ensayo incompleto: falta liberacion');
assert(all(T.pwm(active)==160),'El motor debe permanecer160 durante OA');
du=median(T.u(active));assert(du>0&&max(abs(T.u(active)-du))<1e-4,'Entrada no constante');
assert(all(T.log_perdido==T.log_perdido(1)),'Hubo perdidas de registro');
assert(all(T.huecos(active)==T.huecos(find(active,1))),'Huecos de muestreo durante escalon');
t0=T.t_ms(find(active,1))/1000;
[folder,name]=fileparts(archivo);rawfile=fullfile(folder,[name '.txt']);
if isfile(rawfile)
    events=regexp(fileread(rawfile),'# EVENT t_ms=(\d+) estado=ACTIVE modo=OA','tokens');
    if ~isempty(events),t0=str2double(events{end}{1})/1000;end
else
    warning('Sin TXT de eventos: tiempo del escalon aproximado hasta100ms.');
end
good=T.nuevo==1&T.valido==1&T.cota==0&T.periodo_us>0&isfinite(T.w);
baseline=good&string(T.estado)=="WARM"&T.t_ms/1000>=t0-2;
assert(nnz(baseline)>=3,'Faltan3 periodos de referencia estable');
base=median(2*pi*1e6./(ppr*T.periodo_us(baseline)));
sel=good&active;
assert(nnz(sel)>=6,'Faltan6 periodos nuevos durante escalon; aumenta marcas reales/PPR');
duration=T.periodo_us(sel)/1e6;
finish=T.t_ms(sel)/1000-T.edad_ms(sel)/1000-t0;
begin=finish-duration;
y=2*pi./(ppr*duration);
assert(median(y(end-min(2,numel(y)-1):end))<.97*base,...
    'No hay disminucion suficiente para identificar el efecto del freno');
% Parametros acotados con transformaciones: K<0,tau .01..20s,L0..2s.
decode=@(q)[-exp(max(-12,min(8,q(1)))),.01+19.99./(1+exp(-q(2))),2./(1+exp(-q(3)))];
cost=@(q)sum((averaged(decode(q),begin,finish,base,du)-y).^2);
best=Inf;qbest=[];
kguess=max(.01,(base-median(y(end-2:end)))/du);
for tau0=[.1 .5 1.5 4]
    for delay0=[.02 .3 1]
        q0=[log(kguess),log((tau0-.01)/(20-tau0)),log(delay0/(2-delay0))];
        [q,loss]=fminsearch(cost,q0,optimset('Display','off','MaxIter',2000));
        if loss<best,best=loss;qbest=q;end
    end
end
v=decode(qbest);K=v(1);tau=v(2);L=v(3);
rmse=sqrt(best/numel(y));
fprintf('Modelo EXPLORATORIO OA: K=%.6f rad/s/u; tau=%.4fs; L=%.4fs; RMSE=%.4f rad/s\n',K,tau,L,rmse);
if tau<2*median(duration),warning('Pocos pulsos durante tau: dinamica rapida insuficientemente resuelta.');end
if tau>1.5||L>1.8,warning('Ensayo6s puede no resolver regimen o retardo; ampliar/repetir.');end
tt=linspace(0,6,601)';z=max(0,tt-L);
pred=base+K*du*(1-exp(-z/tau));
figure('Name','Escalon OA medido y modelo');
plot((begin+finish)/2,y*60/(2*pi),'o',tt,pred*60/(2*pi),'-','LineWidth',1.4);
xlabel('Tiempo desde orden de escalon [s]');ylabel('RPM');grid on;
legend('Promedio entre flancos','Modelo puntual','Location','best');
title('Ajuste exploratorio; validar en otro ensayo');
resultado=struct('K',K,'tau_s',tau,'retardo_s',L,'rmse_rad_s',rmse,...
    'periodos_nuevos',numel(y),'ppr',ppr,'experimental_validado',false);
save(fullfile(folder,[name '_modelo.mat']),'resultado');
if exist('tf','file')&&exist('impulse','file')
    G=tf(K,[tau 1],'InputDelay',L);
    figure('Name','Impulso del modelo identificado');impulse(G,6);grid on;
    title('Impulso DEL MODELO: no es un impulso aplicado fisicamente');
    resultado.G=G;save(fullfile(folder,[name '_modelo.mat']),'resultado','G');
else
    warning('Sin Control System Toolbox: falta ejecutar impulse(G). No se declara cumplido ese punto.');
end
end

function a=averaged(v,b,e,base,du)
K=v(1);tau=v(2);L=v(3);
s1=max(0,b-L);s2=max(0,e-L);
H1=s1-tau*(1-exp(-s1/tau));H2=s2-tau*(1-exp(-s2/tau));
a=base+K*du*(H2-H1)./(e-b);
end
