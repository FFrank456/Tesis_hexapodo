%IDENTIFICAR_PATA Identifica las 3 articulaciones de una pata con los CSV del dashboard.
%
% 1. Haz los ensayos del README.md de esta carpeta y exporta un CSV por ensayo
%    (botón Grabar -> Exportar CSV del dashboard). Pon los CSV en la carpeta "datos".
% 2. Edita la lista de archivos de cada articulación abajo y ejecuta este script.
% 3. Copia los comandos sugeridos en el cuadro de comando manual del dashboard.
%
% Funciona con pata_MOD_DRV y con pata_banco1 (mismo formato de CSV).

clear; clc;
carpeta = fullfile(fileparts(mfilename('fullpath')), 'datos');
f = @(nombre) fullfile(carpeta, nombre);

% ---------------------------------------------------------------- Coxa (M1)
% Gira en un plano horizontal: sin término de gravedad.
coxa = identificar_articulacion({f('coxa_rampa.csv'), f('coxa_vaiven.csv')}, 1, ...
                                'gravedad', false, 'fc_vel_hz', 3);

% --------------------------------------------------------------- Tibia (M3)
tibia = identificar_articulacion({f('tibia_rampa.csv'), f('tibia_vaiven.csv')}, 3, ...
                                 'fc_vel_hz', 3);

% --------------------------------------------------------------- Fémur (M2)
% La inercia y la gravedad del fémur cambian con el ángulo de la tibia: identifica
% una postura por archivo y compara G y Jeq.
posturas = {'femur_tibia_estirada.csv', 'femur_tibia_90.csv', 'femur_tibia_recogida.csv'};
femur = {};
for k = 1:numel(posturas)
    if exist(f(posturas{k}), 'file')
        femur{end + 1} = identificar_articulacion({f('femur_rampa.csv'), f(posturas{k})}, 2, ...
                                                  'fc_vel_hz', 3, 'graficar', k == 1); %#ok<SAGROW>
    end
end
if numel(femur) > 1
    G = cellfun(@(p) p.corriente.G, femur);
    J = cellfun(@(p) p.corriente.Jeq, femur);
    fprintf('Fémur según la postura de la tibia:\n');
    for k = 1:numel(femur)
        fprintf('  %-28s G = %.3f A   Jeq = %.5f A*s^2/rad\n', femur{k}.archivos{end}, G(k), J(k));
    end
    variacion = (max(G) - min(G)) / max(mean(G), eps);
    if variacion > 0.3
        fprintf(['  G varía %.0f %%: conviene que GRAV del fémur dependa del ángulo de la tibia\n' ...
                 '  (pídelo en el firmware) o usar el valor de la postura más usada al caminar.\n'], 100 * variacion);
    else
        fprintf('  G varía %.0f %%: basta un solo GRAV (el promedio, %.3f A).\n', 100 * variacion, mean(G));
    end
    % Ganancias con la mayor inercia (la más conservadora).
    [~, kmax] = max(J);
    fprintf('  Para el lazo de velocidad usa los comandos de la postura con mayor Jeq: %s\n', ...
            femur{kmax}.archivos{end});
end
