%% CINEMATICA DIRECTA GEOMETRICA - PATA MODIFICADA (Unidad 2)
% Tesis hexapodo - UTEC
%
% Geometria medida en Fusion (ROBOT_ESAMBLE_alfred_mejorado, 19-nov-2026)
% con el MCP de Fusion. Ver seccion 1 y la validacion de la seccion 4b.
%
% Uso: abrir esta carpeta en MATLAB y ejecutar  simular_cinematica_directa
%
% -------------------------------------------------------------------------
% PLANTEAMIENTO GEOMETRICO
% -------------------------------------------------------------------------
% Pata de 3 GDL: J0 (coxa, eje vertical), J1 (femur) y J2 (tibia), con los
% ejes de J1 y J2 horizontales y paralelos entre si. Por eso, una vez fijado
% el giro de coxa th0, el femur y la tibia se mueven en un PLANO VERTICAL
% (el "plano de la pata"). El problema se separa en dos vistas:
%
%  (a) VISTA LATERAL (plano de la pata): coordenadas r (radial, alejandose
%      del cuerpo) y z (vertical, + arriba), con origen en el eje de J0.
%
%          J1 esta a L1 en r y a h1 en z respecto a J0 (offset fijo de la coxa).
%          El femur mide L2 (J1 -> J2) y forma th1 con la horizontal.
%          La tibia es doblada (tramos rectos). Para la cinematica solo importa
%          la RECTA EQUIVALENTE J2 -> pie:
%              L3    = longitud de esa recta
%              gamma = angulo fijo entre el tramo 1 de la tibia y esa recta
%          La tibia forma -th2 con la prolongacion del femur (th2 + = flexion
%          hacia abajo), asi que la recta J2->pie apunta a
%              phi = th1 - th2 - gamma      (respecto a la horizontal)
%
%          Sumando proyecciones (ley de los vectores en el plano):
%              r = L1 + L2*cos(th1) + L3*cos(th1 - th2 - gamma)
%              z = h1 + L2*sin(th1) + L3*sin(th1 - th2 - gamma)
%
%  (b) VISTA SUPERIOR (planta): el plano de la pata gira th0 alrededor del
%      eje de J0. Si el plano esta desplazado lateralmente una distancia d
%      del eje de J0 (motores no centrados), el punto (r, d) gira:
%              x = r*cos(th0) - d*sin(th0)
%              y = r*sin(th0) + d*cos(th0)
%              z = z
%
%  (c) TIBIA POR TRAMOS -> (L3, gamma). Si la tibia tiene N tramos de
%      longitud l_k a un angulo a_k respecto al tramo 1 (negativo = abajo):
%              u = sum l_k*cos(a_k)       (a lo largo del tramo 1)
%              w = -sum l_k*sin(a_k)      (perpendicular, + hacia abajo)
%              L3 = sqrt(u^2 + w^2),   gamma = atan2(w, u)
%
%  (d) ENCODER -> MODELO. El firmware entrega angulos de encoder q. El
%      modelo usa th = signo.*q + offset, donde signo corrige el sentido de
%      giro y offset es el angulo del modelo cuando el encoder marca 0.
%
%  (e) MARCO DEL CUERPO (opcional): p_cuerpo = p_montaje + Rz(psi_m)*p_J0,
%      con p_montaje la posicion del eje J0 en el cuerpo y psi_m el angulo
%      de montaje de la pata en planta.
% -------------------------------------------------------------------------

clear; clc;

%% 1) GEOMETRIA MEDIDA EN FUSION  [mm]  (pose cero: J0 = J1 = J2 = 0)
% Puntos en el marco raiz de Fusion, leidos con el MCP de Fusion. Se
% comprobo que cada eje de junta coincide (0.000 mm) con el eje D de 3 mm,
% la carcasa del N20, la brida y el buje de su motor.
cad.J0    = [ 97.557;   34.413;  -9.703];  % punto del eje J0 (vertical)
cad.J1    = [145.675;   11.821;  -3.653];  % punto del eje J1
cad.J2    = [188.031;  -30.676;  -3.653];  % punto del eje J2
cad.pie   = [244.504; -112.635; -40.009];  % punto mas bajo de soporte_tibia_A
cad.ejeJ1 = [0.70711; 0.70711; 0];         % direccion de los ejes J1 y J2 (paralelos)

% Base del plano de la pata: e_r radial, e_a lateral (eje de J1/J2), e_z arriba
e_a = cad.ejeJ1 / norm(cad.ejeJ1);
e_z = [0; 0; 1];
e_r = cross(e_a, e_z);
Rm  = [e_r e_a e_z];                       % marco J0 del modelo -> marco Fusion
enPlano = @(p) Rm' * (p - cad.J0);         % (r, a, z) respecto al eje de J0

j1 = enPlano(cad.J1); j2 = enPlano(cad.J2); pf = enPlano(cad.pie);
g.L1 = j1(1);                              % J0 -> J1 radial
g.h1 = j1(3);                              % J0 -> J1 altura (+ J1 mas arriba)
g.L2 = hypot(j2(1) - j1(1), j2(3) - j1(3));% J1 -> J2
femur0 = atan2d(j2(3) - j1(3), j2(1) - j1(1));
assert(abs(femur0) < 0.01, 'El femur no esta horizontal en la pose cero (%.3f deg).', femur0);
g.d  = pf(2);                              % desfase lateral del pie respecto al eje J0

%% 2) CONVENCIONES DE ANGULO
% Fusion -> modelo (verificado girando cada junta +10 deg en el CAD):
%   J0 + en Fusion = horario visto desde arriba  -> th0 = -qF0
%   J1 + en Fusion = el femur sube               -> th1 = +qF1
%   J2 + en Fusion = la tibia sube (se abre)     -> th2 = -qF2
fus.signo = [-1  1 -1];

% Encoder -> modelo: th = enc.signo .* q_encoder + enc.offset
%   Femur: encoder negativo = SUBE -> th1 = -q1   (confirmado a mano el 19-nov y por la
%          identificacion: el femur cuelga en q1 = +1.47 rad)
%   Tibia: encoder negativo = SUBE -> th2 = +q2   (confirmado a mano el 19-nov;
%          th2 + es flexion hacia abajo)
%   Coxa : encoder negativo = HORARIO visto desde arriba -> th0 = +q0 (confirmado a mano
%          el 20-nov; th0 + es antihorario)
%   (Comparado con Fusion: en femur y tibia el encoder gira al reves que la junta del CAD.)
enc.signo  = [ 1  -1   1];
enc.offset = deg2rad([0 0 0]); % supone encoder = 0 en la pose cero del CAD (por confirmar)

% Limites de trabajo en angulos de MODELO [deg] (nota de rediseno: +-90 en las tres)
lim = [-90 90;     % J0
       -90 90;     % J1
       -90 90];    % J2

% Marco de salida adicional: el marco raiz de Fusion
montaje.p   = cad.J0;
montaje.psi = atan2d(e_r(2), e_r(1));      % -45 deg

%% 3) TIBIA EQUIVALENTE (L3, gamma): recta J2 -> pie en la pose cero
u = pf(1) - j2(1);                         % avance a lo largo del femur
w = -(pf(3) - j2(3));                      % caida (+ hacia abajo)
g.L3    = hypot(u, w);
g.gamma = atan2(w, u);
g.tramos = [g.L3, -rad2deg(g.gamma)];      % para dibujar: la recta equivalente
fprintf('L1 = %.3f  h1 = %.3f  L2 = %.3f  d = %.3f mm\n', g.L1, g.h1, g.L2, g.d);
fprintf('Tibia: u = %.3f  w = %.3f  ->  L3 = %.3f mm, gamma = %.3f deg\n', ...
        u, w, g.L3, rad2deg(g.gamma));

%% 4) COMPROBACION: geometria vs transformaciones homogeneas
% Misma cadena escrita con matrices (Rz, Ry, traslaciones). No valida las
% medidas, solo que las ecuaciones geometricas estan bien escritas.
Rz = @(a) [cos(a) -sin(a) 0 0; sin(a) cos(a) 0 0; 0 0 1 0; 0 0 0 1];
Ry = @(a) [cos(a) 0 sin(a) 0; 0 1 0 0; -sin(a) 0 cos(a) 0; 0 0 0 1];
Tr = @(x,y,z) [eye(3) [x;y;z]; 0 0 0 1];
fkH = @(t) Rz(t(1)) * Tr(g.L1, g.d, g.h1) * Ry(-t(2)) * Tr(g.L2,0,0) ...
           * Ry(t(3) + g.gamma) * Tr(g.L3,0,0);

rng(1); errMax = 0; errTramos = 0;
for k = 1:500
    t = deg2rad(lim(:,1) + (lim(:,2)-lim(:,1)).*rand(3,1)).';
    [p, ~, tib] = cd_pata(t, g);
    T = fkH(t);
    errMax    = max(errMax, norm(p - T(1:3,4)));
    errTramos = max(errTramos, norm(p - tib(:,end)));
end
fprintf('Error geometria vs matrices homogeneas: %.2e mm\n', errMax);
fprintf('Error recta equivalente vs tramos:      %.2e mm\n', errTramos);
assert(errMax < 1e-9 && errTramos < 1e-9, 'Las dos formulaciones no coinciden.');

%% 4b) VALIDACION CONTRA FUSION
% Cada junta se giro +10 deg en Fusion y se midio el mismo punto del pie.
% Columnas: [qF0 qF1 qF2] en deg | pie en el marco raiz de Fusion [mm]
val = [ 0  0  0   244.504 -112.635 -40.009
       10  0  0   216.737 -135.918 -40.009
        0 10  0   247.272 -115.403 -12.040
        0  0 10   247.917 -116.048 -22.459
       20 -15 25  186.524 -156.547 -37.988     % poses combinadas
      -35 40 -60  270.389    3.866 -32.728];
fprintf('\nValidacion contra Fusion (angulos de Fusion):\n');
fprintf('  qF0  qF1  qF2 |  error [mm]\n');
errFus = zeros(size(val,1),1);
for k = 1:size(val,1)
    th = fus.signo .* deg2rad(val(k,1:3));
    pF = cad.J0 + Rm * cd_pata(th, g);
    errFus(k) = norm(pF - val(k,4:6).');
    fprintf('%5.0f%5.0f%5.0f | %8.4f\n', val(k,1:3), errFus(k));
end
assert(max(errFus) < 0.05, 'El modelo no reproduce las poses de Fusion.');

%% 5) POSE CERO Y EJEMPLOS
poses = [0 0 0; 30 0 0; 0 30 0; 0 0 30; 20 -20 40];   % [deg] modelo
fprintf('\n   th0    th1    th2  |      x        y        z   [mm, marco J0]\n');
for k = 1:size(poses,1)
    p = cd_pata(deg2rad(poses(k,:)), g);
    fprintf('%6.1f %6.1f %6.1f  | %8.2f %8.2f %8.2f\n', poses(k,:), p);
end

%% 6) ESPACIO DE TRABAJO (vista lateral, th0 = 0)
n = 61;
[A1, A2] = meshgrid(linspace(lim(2,1), lim(2,2), n), linspace(lim(3,1), lim(3,2), n));
R = zeros(size(A1)); Z = R;
for k = 1:numel(A1)
    p = cd_pata(deg2rad([0 A1(k) A2(k)]), g);
    R(k) = p(1); Z(k) = p(3);
end

if ~usejava('desktop') && isempty(getenv('CD_FORZAR_GUI'))
    fprintf('\nSin escritorio de MATLAB: se omiten las figuras.\n');
    return
end

figure('Name', 'Espacio de trabajo (plano de la pata)');
scatter(R(:), Z(:), 4, A2(:), 'filled'); axis equal; grid on;
xlabel('r [mm]'); ylabel('z [mm]'); cb = colorbar; cb.Label.String = '\theta_2 [deg]';
title('Pie alcanzable con \theta_0 = 0 (color = \theta_2)');

%% 7) SIMULACION INTERACTIVA
fig = uifigure('Name', 'Cinematica directa - pata modificada', 'Position', [100 100 1100 600]);
gl  = uigridlayout(fig, [2 3]); gl.RowHeight = {'1x', 150}; gl.ColumnWidth = {'1x', '1x', 280};
ax3 = uiaxes(gl); ax3.Layout.Row = 1; ax3.Layout.Column = 1;
axL = uiaxes(gl); axL.Layout.Row = 1; axL.Layout.Column = 2;
pan = uipanel(gl, 'Title', 'Articulaciones (modelo, deg)'); pan.Layout.Row = [1 2]; pan.Layout.Column = 3;
txt = uitextarea(gl, 'Editable', 'off', 'FontName', 'Consolas'); txt.Layout.Row = 2; txt.Layout.Column = [1 2];

nombres = {'J0 coxa  th0 (+ antihorario)', 'J1 femur th1 (+ sube)', 'J2 tibia th2 (+ baja)'};
sl = gobjects(1,3); ef = gobjects(1,3);
for j = 1:3
    y0 = 540 - 150*(j-1);
    uilabel(pan, 'Text', nombres{j}, 'Position', [10 y0 260 22], 'FontWeight', 'bold');
    sl(j) = uislider(pan, 'Limits', lim(j,:), 'Value', 0, 'Position', [15 y0-15 240 3]);
    uilabel(pan, 'Text', 'Encoder [rad]:', 'Position', [10 y0-90 100 22]);
    ef(j) = uieditfield(pan, 'numeric', 'Value', 0, 'ValueDisplayFormat', '%.3f', ...
                        'Position', [110 y0-90 90 22]);
end
btn = uibutton(pan, 'Text', 'Volver a cero', 'Position', [15 60 240 30]);
ui = struct('sl', sl, 'ef', ef, 'ax3', ax3, 'axL', axL, 'txt', txt, ...
            'alcance', g.L1 + g.L2 + g.L3 + abs(g.d) + 10);
for j = 1:3
    sl(j).ValueChangingFcn = @(s,e) moverSlider(j, e.Value, ui, g, enc, montaje);
    sl(j).ValueChangedFcn  = @(s,e) moverSlider(j, e.Value, ui, g, enc, montaje);
    ef(j).ValueChangedFcn  = @(s,e) escribirEncoder(j, e.Value, ui, g, enc, montaje, lim);
end
btn.ButtonPushedFcn =@(~,~) ponerCero(ui, g, enc, montaje);
ponerCero(ui, g, enc, montaje);

%% ---------------- funciones locales ----------------
function moverSlider(j, valor, ui, g, enc, montaje)
    th = [ui.sl.Value]; th(j) = valor;
    refrescar(deg2rad(th), ui, g, enc, montaje);
end

function escribirEncoder(j, q, ui, g, enc, montaje, lim)
    thj = rad2deg(enc.signo(j)*q + enc.offset(j));
    if thj < lim(j,1) || thj > lim(j,2)
        uialert(ancestor(ui.sl(j), 'figure'), sprintf('Fuera de limites: th = %.1f deg', thj), 'Limite');
        thj = min(max(thj, lim(j,1)), lim(j,2));
    end
    ui.sl(j).Value = thj;
    refrescar(deg2rad([ui.sl.Value]), ui, g, enc, montaje);
end

function ponerCero(ui, g, enc, montaje)
    for j = 1:3, ui.sl(j).Value = 0; end
    refrescar([0 0 0], ui, g, enc, montaje);
end

function refrescar(th, ui, g, enc, montaje)
    q = (th - enc.offset) ./ enc.signo;
    for j = 1:3, ui.ef(j).Value = q(j); end
    dibujarPata(th, g, enc, montaje, ui.ax3, ui.axL, ui.txt, ui.alcance);
end

function dibujarPata(th, g, enc, montaje, ax3, axL, txt, alcance)
    [p, P, tib] = cd_pata(th, g);
    [~, P0, tib0] = cd_pata([0 0 0], g);

    % Vista 3D (marco J0)
    cla(ax3); hold(ax3, 'on');
    gris = [0.6 0.6 0.6];
    plot3(ax3, P0(1,1:3), P0(2,1:3), P0(3,1:3), '--', 'Color', gris);
    plot3(ax3, tib0(1,:), tib0(2,:), tib0(3,:), '--', 'Color', gris);
    plot3(ax3, P(1,1:3), P(2,1:3), P(3,1:3), '-o', 'LineWidth', 3, 'Color', [0.1 0.4 0.8]);
    plot3(ax3, tib(1,:), tib(2,:), tib(3,:), '-', 'LineWidth', 3, 'Color', [0.85 0.4 0.1]);
    plot3(ax3, [P(1,3) p(1)], [P(2,3) p(2)], [P(3,3) p(3)], ':', 'Color', [0.5 0.5 0.5]);
    plot3(ax3, p(1), p(2), p(3), 'p', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    axis(ax3, 'equal'); grid(ax3, 'on'); view(ax3, 35, 25);
    xlim(ax3, [-alcance alcance]); ylim(ax3, [-alcance alcance]); zlim(ax3, [-alcance alcance]);
    xlabel(ax3, 'x [mm]'); ylabel(ax3, 'y [mm]'); zlabel(ax3, 'z [mm]');
    title(ax3, 'Vista 3D (marco J0)'); hold(ax3, 'off');

    % Vista lateral en el plano de la pata
    radial = @(Q) cos(th(1))*Q(1,:) + sin(th(1))*Q(2,:);
    cla(axL); hold(axL, 'on');
    plot(axL, radial(P(:,1:3)), P(3,1:3), '-o', 'LineWidth', 3, 'Color', [0.1 0.4 0.8]);
    plot(axL, radial(tib), tib(3,:), '-', 'LineWidth', 3, 'Color', [0.85 0.4 0.1]);
    plot(axL, radial([P(:,3) p]), [P(3,3) p(3)], ':', 'Color', [0.5 0.5 0.5]);
    plot(axL, radial(p), p(3), 'p', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    yline(axL, 0, 'k:');
    axis(axL, 'equal'); grid(axL, 'on');
    xlim(axL, [-20 alcance]); ylim(axL, [-alcance alcance]);
    xlabel(axL, 'r [mm]'); ylabel(axL, 'z [mm]'); title(axL, 'Vista lateral (plano de la pata)');
    hold(axL, 'off');

    % Texto: modelo, encoder y cuerpo
    q   = (th - enc.offset) ./ enc.signo;
    psi = deg2rad(montaje.psi);
    pc  = montaje.p + [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1] * p;
    txt.Value = {
        sprintf('Modelo  th = [%7.2f %7.2f %7.2f] deg', rad2deg(th))
        sprintf('Encoder q  = [%7.3f %7.3f %7.3f] rad', q)
        sprintf('Pie (marco J0)     = [%8.2f %8.2f %8.2f] mm', p)
        sprintf('Pie (marco Fusion) = [%8.2f %8.2f %8.2f] mm', pc)
        sprintf('L1=%.2f h1=%.2f d=%.2f L2=%.2f L3=%.2f gamma=%.2f deg   (Fusion)', ...
                g.L1, g.h1, g.d, g.L2, g.L3, rad2deg(g.gamma))};
end
