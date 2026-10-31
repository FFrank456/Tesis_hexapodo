%% Identificacion PWM (%) -> corriente CS equivalente (A)
% Requiere System Identification Toolbox.
% Ejecutar el script completo. No controla el motor ni modifica el firmware.
% Conserva todas las muestras y transitorios: no aplica EMA ni recortes.
% El promedio CS del DRV8876 en drive/coast no es una medida validada de torque.

archivoEstimacion = 'D:/matlab_tesis/MultiSTEP_4.mat';
archivoValidacion = ''; % Otro ensayo independiente; vacio = aun sin validacion.

% MultiSTEP_4 comienza con el motor detenido y corriente cero.
% Usar 'estimate' si otro registro comienza con estados iniciales desconocidos.
condicionInicial = 'zero';
assert(ismember(condicionInicial,{'zero','estimate'}), ...
    'condicionInicial debe ser zero o estimate.');

%% Cargar y verificar el tiempo del ESP32 (no el tiempo de recepcion del PC)
[datCurrent, tiempo, u, y] = cargarEnsayo(archivoEstimacion);
Ts = datCurrent.Ts;
fprintf('Estimacion: %d muestras, %.6f s entre muestras, %.3f s de registro.\n', ...
    numel(y), Ts, tiempo(end));
fprintf('Entrada: PWM de 0 a 100 %% (sin normalizar). Salida: A equivalentes CS.\n');

figure('Name','Datos de estimacion');
subplot(2,1,1);
stairs(tiempo,u); grid on; ylabel('PWM (%)');
title('Entrada aplicada');
subplot(2,1,2);
plot(tiempo,y); grid on; ylabel('CS equivalente (A)'); xlabel('Tiempo (s)');
title('Media DMA, sin filtrado adicional');

%% Modelos continuos estrictamente propios: numero de ceros < numero de polos
% tfest(datos,np,nz): np = polos; nz = ceros.
% Se excluyen G11 y G22 para probar modelos sin salto algebraico instantaneo.
% Esto no impide transitorios negativos ni garantiza un buen ajuste global.
opciones = tfestOptions('InitialCondition',condicionInicial,'Display','off');
G10 = tfest(datCurrent,1,0,opciones);
G20 = tfest(datCurrent,2,0,opciones);
G21 = tfest(datCurrent,2,1,opciones);
G10.Name = 'G10: 1 polo, 0 ceros';
G20.Name = 'G20: 2 polos, 0 ceros';
G21.Name = 'G21: 2 polos, 1 cero';
modelos = {G10,G20,G21};

%% Comparacion por simulacion sobre el mismo ensayo: NO es validacion independiente
% Las condiciones iniciales se tratan igual en estimacion y comparacion.
% compareOptions usa 'z'/'e'; tfestOptions usa 'zero'/'estimate'.
opComparar = compareOptions('InitialCondition',condicionInicial(1));
figure('Name','Ajuste sobre datos de estimacion');
compare(datCurrent,G10,G20,G21,opComparar);
resumenEstimacion = evaluarModelos(datCurrent,modelos,opComparar);
disp('Ajuste sobre el ensayo utilizado para estimar (no demuestra generalizacion):');
disp(resumenEstimacion);
disp('Transferencias, con entrada en puntos porcentuales de PWM:');
disp(G10); disp(G20); disp(G21);

%% Validacion opcional con otro ensayo completo y comparable
if ~isempty(archivoValidacion)
    assert(~strcmpi(archivoEstimacion,archivoValidacion), ...
        'Selecciona otro ensayo para validacion.');
    [datValidacion,~,~,~] = cargarEnsayo(archivoValidacion);
    % Usar aqui un ensayo que tambien comience en reposo cuando se elige zero.
    figure('Name','Validacion independiente por simulacion');
    compare(datValidacion,G10,G20,G21,opComparar);
    resumenValidacion = evaluarModelos(datValidacion,modelos,opComparar);
    disp('Validacion independiente:');
    disp(resumenValidacion);
    for k = 1:numel(modelos)
        figure('Name',['Residuos de validacion: ' modelos{k}.Name]);
        resid(datValidacion,modelos{k});
    end
else
    fprintf(['Sin ensayo independiente: no se selecciona automaticamente un controlador.\n' ...
        'Define archivoValidacion para comparar y analizar residuos en otro ensayo.\n']);
end

%% Funciones auxiliares
function [datos,t,u,y] = cargarEnsayo(archivo)
% Lee una tabla MAT con time (s del ESP32), pwm (%) y currentMEAN (A eq.).
% Rechaza huecos/NaN; no borra filas ni comprime el eje temporal para ocultarlos.
    contenido = load(archivo);
    nombres = fieldnames(contenido);
    candidatas = {};
    for k = 1:numel(nombres)
        valor = contenido.(nombres{k});
        if istable(valor) && all(ismember({'time','pwm','currentMEAN'}, ...
                valor.Properties.VariableNames))
            candidatas{end+1} = nombres{k}; %#ok<AGROW>
        end
    end
    assert(numel(candidatas)==1, ...
        'El MAT debe contener una sola tabla con time, pwm y currentMEAN.');
    tabla = contenido.(candidatas{1});
    t = double(tabla.time(:));
    u = double(tabla.pwm(:));
    y = double(tabla.currentMEAN(:));
    assert(numel(t)>=20 && numel(u)==numel(t) && numel(y)==numel(t), ...
        'Longitudes incompatibles o muy pocas muestras.');
    assert(all(isfinite(t)) && all(isfinite(u)) && all(isfinite(y)), ...
        'Hay datos no finitos: revisar adquisicion antes de identificar.');
    assert(all(abs(u)<=100), 'PWM fuera de -100 a 100 por ciento.');
    dt = diff(t);
    assert(all(dt>0), 'Tiempos repetidos o fuera de orden.');
    TsLocal = median(dt);
    assert(all(abs(dt-TsLocal)<=1e-6), ...
        'Muestreo irregular: revisar huecos; no usar solo el primer intervalo.');
    assert(max(u)>min(u) && max(y)>min(y), 'Entrada o salida sin variacion.');
    t = t-t(1);
    datos = iddata(y,u,TsLocal);
    datos.InterSample = 'zoh'; % PWM constante entre cambios de consigna.
    datos.TimeUnit = 'seconds';
    datos.InputName = {'PWM'};
    datos.InputUnit = {'%'};
    datos.OutputName = {'CS_media_DMA'};
    datos.OutputUnit = {'A equivalentes'};
end

function resumen = evaluarModelos(datos,modelos,opciones)
% FIT NRMSE y RMSE de simulacion. Sin recortar salidas negativas del modelo.
    n = numel(modelos);
    Modelo = cell(n,1); Fit_porcentaje = zeros(n,1);
    RMSE_A = zeros(n,1); Estable = false(n,1); Ganancia_A_por_pct = zeros(n,1);
    for k = 1:n
        [simulado,ajuste] = compare(datos,modelos{k},opciones);
        error = datos.OutputData-simulado.OutputData;
        Modelo{k} = modelos{k}.Name;
        Fit_porcentaje(k) = ajuste;
        RMSE_A(k) = sqrt(mean(error.^2));
        Estable(k) = isstable(modelos{k});
        Ganancia_A_por_pct(k) = dcgain(modelos{k});
    end
    resumen = table(Modelo,Fit_porcentaje,RMSE_A,Estable,Ganancia_A_por_pct);
end
