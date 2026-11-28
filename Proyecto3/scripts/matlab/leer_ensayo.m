function [T, eventos] = leer_ensayo(archivo)
% LEER_ENSAYO  Lee un CSV de ensayo con el formato de Registro.h.
% Responsable: Integrante B (lo usan todos los scripts).
% T.t contiene el tiempo en segundos. El segundo resultado conserva las
% lineas con '#' en el orden de captura.
%
% Uso: T = leer_ensayo('../../datos/identificacion/oa_u050_r1.csv');

% Localiza el encabezado aunque el puerto haya emitido eventos al arrancar.
nombres = ["t_ms", "modo", "ref", "w", "e", "u", "pwm", "nueva"];
lineas = strip(readlines(archivo));
comentarios = startsWith(lineas, "#");
eventos = lineas(comentarios);
encabezado = find(lineas ~= "" & ~comentarios, 1);
if isempty(encabezado) || lineas(encabezado) ~= join(nombres, ",")
    error('leer_ensayo:Formato', 'Encabezado CSV incompatible con Registro.h.');
end

% Los tipos fijos evitan inferencias distintas cuando cambia el ensayo.
opciones = delimitedTextImportOptions('NumVariables', numel(nombres));
opciones.Delimiter = ',';
opciones.CommentStyle = '#';
opciones.DataLines = [encabezado + 1, Inf];
opciones.VariableNames = nombres;
opciones.VariableTypes = ["double", "string", repmat("double", 1, 6)];
opciones.ExtraColumnsRule = 'error';
opciones.ImportErrorRule = 'error';
opciones.MissingRule = 'error';
T = readtable(archivo, opciones);

% Rechaza capturas vacias, mezcladas o con valores fuera del formato.
numeros = T{:, nombres(nombres ~= "modo")};
if isempty(T) || any(~isfinite(numeros), 'all') || ...
        any(~ismember(T.modo, ["OA", "PID"])) || any(T.modo ~= T.modo(1)) || ...
        any(T.t_ms < 0 | mod(T.t_ms, 1) ~= 0) || any(diff(T.t_ms) <= 0) || ...
        any(T.u < 0 | T.u > 1) || any(T.pwm < 0 | T.pwm > 255 | mod(T.pwm, 1) ~= 0) || ...
        any(~ismember(T.nueva, [0, 1]))
    error('leer_ensayo:Datos', 'El CSV no contiene un ensayo valido.');
end
T.t = T.t_ms / 1000;
end
