function [g, enc] = geometria_pata()
% GEOMETRIA_PATA  Geometria de la pata modificada y conversion encoder <-> modelo.
%
%   [g, enc] = geometria_pata()
%
%   g   : L1, h1, d, L2, L3, gamma, tramos (mm, rad), medidos en Fusion
%         (ROBOT_ESAMBLE_alfred_mejorado, 19-nov-2026). Son los mismos valores que
%         simular_cinematica_directa.m calcula desde el CAD (ver README.md).
%   enc : .signo, .offset  ->  th_modelo = enc.signo .* q_encoder + enc.offset
%                              q_encoder = (th_modelo - enc.offset) ./ enc.signo
%         .lim  limites de trabajo en ENCODER [rad] (3x2: coxa, femur, tibia)
%
%   Signos confirmados a mano (19 y 20-nov): coxa encoder - = horario visto desde
%   arriba; femur y tibia encoder + = bajan. Offset 0: el cero se pone en la pose
%   cero del CAD (femur horizontal, tibia alineada, coxa centrada).

    g.L1 = 50.000;                 % J0 -> J1, radial
    g.h1 = 6.050;                  % J1 sobre J0
    g.d  = -0.071;                 % desfase lateral del pie
    g.L2 = 60.000;                 % J1 -> J2 (femur)
    g.L3 = hypot(97.886, 36.356);  % recta equivalente J2 -> pie (104.420)
    g.gamma = atan2(36.356, 97.886); % 20.376 deg
    g.tramos = [g.L3, -rad2deg(g.gamma)];

    enc.signo  = [1 -1 1];
    enc.offset = [0 0 0];
    % PROVISIONALES hasta medir los rangos a mano (paso 1 de la cinematica):
    %   coxa  : +-0.6 (vaiven de la identificacion del 19-nov)
    %   femur : +-1.2 (pruebas y calibracion del 19/20-nov, con margen)
    %   tibia : +-1.6 (topes en +-1.74, 19/20-nov)
    enc.lim = [-0.6 0.6; -1.2 1.2; -1.6 1.6];
end
