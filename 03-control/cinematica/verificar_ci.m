% VERIFICAR_CI  Comprueba la cinematica inversa (ci_pata) contra la directa (cd_pata)
% y dibuja el espacio de trabajo alcanzable con los limites de los encoders.
%
%   En MATLAB, desde esta carpeta:  verificar_ci
%   Desde PowerShell (raiz de TESIS_2):
%     matlab -wait -batch "cd('03-control/cinematica'); verificar_ci"
%
%   Funciona en MATLAB y en GNU Octave, sin toolboxes.

[g, enc] = geometria_pata();
q2th = @(q) enc.signo(:) .* q + enc.offset(:);       % encoder -> modelo (3xN)
th2q = @(th) (th - enc.offset(:)) ./ enc.signo(:);    % modelo -> encoder (3xN)
fk = @(th) cd_pata_n(th, g);

%% 1) Ida y vuelta: angulos -> pie -> angulos, en todo el rango de los encoders
rng(1);
N = 20000;
q = enc.lim(:, 1) + (enc.lim(:, 2) - enc.lim(:, 1)) .* rand(3, N);
th = q2th(q);
p = fk(th);
rama = sign(th(3, :) + g.gamma); rama(rama == 0) = 1;
thR = zeros(3, N); okR = false(1, N);
for s = [1 -1]
    k = rama == s;
    [thR(:, k), okR(k)] = ci_pata(p(:, k), g, s);
end
eAng = max(abs(atan2(sin(thR - th), cos(thR - th))), [], 1);
ePos = sqrt(sum((fk(thR) - p).^2, 1));
fprintf('1) Ida y vuelta, %d poses en el rango de los encoders:\n', N);
fprintf('   alcanzables %d/%d | error angulo max %.2e rad | error pie max %.2e mm\n', ...
        sum(okR), N, max(eAng), max(ePos));
fprintf('   rama de la rodilla: %d poses con la tibia doblada hacia abajo (+1), %d hacia arriba (-1)\n', ...
        sum(rama == 1), sum(rama == -1));
assert(all(okR) && max(eAng) < 1e-9 && max(ePos) < 1e-9, 'La inversa no reproduce la directa.');

%% 2) Poses de validacion contra Fusion (angulos de Fusion en grados)
fus = [0 0 0; 10 0 0; 0 10 0; 0 0 10; 20 -15 25; -35 40 -60];
fprintf('2) Poses validadas en Fusion (pie -> angulos -> pie):\n');
for k = 1:size(fus, 1)
    t = deg2rad(fus(k, :))' .* [-1; 1; -1];          % Fusion -> modelo
    pk = fk(t);
    s = sign(t(3) + g.gamma); if s == 0, s = 1; end
    tk = ci_pata(pk, g, s);
    fprintf('   Fusion (%4.0f,%4.0f,%4.0f) deg -> encoder (%+.3f, %+.3f, %+.3f) rad, error %.1e rad\n', ...
            fus(k, :), th2q(tk), max(abs(tk - t)));
end

%% 3) Puntos fuera de alcance
lejos = [400; 0; 0];  cerca = [55; 0; 6];
[~, ok1, i1] = ci_pata(lejos, g); [~, ok2, i2] = ci_pata(cerca, g);
fprintf('3) Fuera de alcance: (400,0,0) ok=%d c=%.2f | (55,0,6) ok=%d c=%.2f\n', ok1, i1.c, ok2, i2.c);
assert(~ok1 && ~ok2, 'Deberia marcar los puntos inalcanzables.');

%% 4) Espacio de trabajo en el plano de la pata (coxa = 0), rama normal
paso = 2;                                              % mm
[R, Z] = meshgrid(0:paso:240, -200:paso:160);
P = [R(:)'; zeros(1, numel(R)); Z(:)'];
[thW, okW] = ci_pata(P, g, 1);
qW = th2q(thW);
enRango = okW & all(qW >= enc.lim(:, 1) & qW <= enc.lim(:, 2), 1);
fprintf('4) Espacio de trabajo (coxa = 0, rama normal, limites provisionales):\n');
fprintf('   alcance geometrico %.0f cm^2, dentro de los limites de los encoders %.0f cm^2\n', ...
        sum(okW) * paso^2 / 100, sum(enRango) * paso^2 / 100);
zr = Z(enRango); rr = R(enRango);
fprintf('   altura del pie dentro de los limites: %.0f a %.0f mm (respecto de J0)\n', min(zr), max(zr));
fprintf('   alcance radial: %.0f a %.0f mm\n', min(rr), max(rr));

if usejava('desktop') || ~isempty(getenv('DISPLAY')) || ispc
    fig = figure('Name', 'Espacio de trabajo - cinematica inversa', 'Color', 'w', 'Visible', 'off');
    if exist('theme', 'file') == 2 || exist('theme', 'builtin') == 5, theme(fig, 'light'); end
    hold on; axis equal; grid on;
    plot(R(okW & ~enRango), Z(okW & ~enRango), '.', 'Color', [0.85 0.85 0.85], 'MarkerSize', 4);
    plot(rr, zr, '.', 'Color', [0.2 0.45 0.8], 'MarkerSize', 4);
    [~, P0] = cd_pata([0 0 0], g);
    plot(hypot(P0(1, :), P0(2, :)), P0(3, :), 'k-o', 'LineWidth', 2, 'MarkerFaceColor', 'k');
    xlabel('r: distancia al eje de la coxa (mm)'); ylabel('z: altura respecto de J0 (mm)');
    title('Pie alcanzable (coxa = 0, rodilla normal)');
    legend({'alcanzable fuera de los limites', 'dentro de los limites de los encoders', 'pose cero'}, ...
           'Location', 'southwest');
    print(fig, fullfile(fileparts(mfilename('fullpath')), 'espacio_trabajo_ci.png'), '-dpng', '-r120');
    close(fig);
    fprintf('   figura: espacio_trabajo_ci.png\n');
end

%% 5) Ejemplo: pie bajo la pata, en posicion de apoyo
objetivo = [150; 0; -110];
[tO, okO] = ci_pata(objetivo, g);
qO = th2q(tO);
fprintf('5) Pie en (%.0f, %.0f, %.0f) mm -> encoder (%+.3f, %+.3f, %+.3f) rad, alcanzable=%d, en rango=%d\n', ...
        objetivo, qO, okO, all(qO >= enc.lim(:, 1) & qO <= enc.lim(:, 2)));
fprintf('Cinematica inversa verificada.\n');

function p = cd_pata_n(th, g)
% cd_pata para varias columnas
    p = zeros(3, size(th, 2));
    for k = 1:size(th, 2), p(:, k) = cd_pata(th(:, k)', g); end
end
