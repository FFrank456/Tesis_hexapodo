function D = leer_csv_pata(archivo)
%LEER_CSV_PATA Lee un CSV exportado por el dashboard de pata_MOD_DRV / pata_banco1.
%
%   D = leer_csv_pata('pata_MOD_DRV_20261111_190500.csv')
%
%   El CSV tiene la cabecera MHEADER del firmware (TEL,MULTI, 100 Hz):
%     time_ms, selected, y para cada motor i = 1..3:
%     mi_mode, mi_duty_percent, mi_current_a, mi_current_ref_a, mi_position_rad,
%     mi_speed_rad_s, mi_speed_ref_rad_s, mi_position_ref_rad, mi_position_state,
%     mi_current_status, mi_flags
%
%   Devuelve:
%     D.t          tiempo en s (desde la primera muestra)
%     D.m(i)       struct por motor con los campos mode, duty, cur, curRef, pos,
%                  spd, spdRef, posRef, state, status, flags (vectores columna)
%     D.archivo    nombre del archivo
%
%   Funciona en MATLAB y en GNU Octave (sin toolboxes).

    texto = fileread(archivo);
    lineas = regexp(texto, '\r?\n', 'split');
    lineas = lineas(~cellfun(@isempty, strtrim(lineas)));
    if isempty(lineas)
        error('leer_csv_pata:vacio', 'El archivo %s está vacío.', archivo);
    end
    cabecera = strtrim(strsplit(lineas{1}, ','));
    n = numel(cabecera);
    datos = nan(numel(lineas) - 1, n);
    fila = 0;
    for k = 2:numel(lineas)
        campos = strsplit(lineas{k}, ',');
        if numel(campos) ~= n, continue; end          % línea cortada o corrupta
        v = str2double(campos);                        % 'nan' -> NaN
        fila = fila + 1;
        datos(fila, :) = v;
    end
    datos = datos(1:fila, :);
    if fila < 10
        error('leer_csv_pata:corto', 'El archivo %s tiene solo %d muestras válidas.', archivo, fila);
    end

    col = @(nombre) columna(cabecera, nombre);
    t_ms = datos(:, col('time_ms'));
    % Quitar muestras duplicadas o fuera de orden (reconexiones).
    [t_ms, orden] = unique(t_ms);
    datos = datos(orden, :);
    D.t = (t_ms - t_ms(1)) / 1000;
    D.archivo = archivo;

    nombres = {'mode', 'duty_percent', 'current_a', 'current_ref_a', 'position_rad', ...
               'speed_rad_s', 'speed_ref_rad_s', 'position_ref_rad', 'position_state', ...
               'current_status', 'flags'};
    campos = {'mode', 'duty', 'cur', 'curRef', 'pos', 'spd', 'spdRef', 'posRef', ...
              'state', 'status', 'flags'};
    for i = 1:3
        for c = 1:numel(nombres)
            m.(campos{c}) = datos(:, col(sprintf('m%d_%s', i, nombres{c})));
        end
        D.m(i) = m; %#ok<AGROW>
    end
end

function j = columna(cabecera, nombre)
    j = find(strcmp(cabecera, nombre), 1);
    if isempty(j)
        error('leer_csv_pata:columna', ...
              'Falta la columna "%s". ¿El CSV salió del dashboard en modo TEL,MULTI?', nombre);
    end
end
