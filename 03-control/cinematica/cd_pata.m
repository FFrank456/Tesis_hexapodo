function [pie, P, tibia] = cd_pata(th, g)
% CD_PATA  Cinematica directa geometrica de una pata de 3 GDL (coxa-femur-tibia).
%
%   [pie, P, tibia] = cd_pata(th, g)
%
%   th    = [th0 th1 th2] angulos DEL MODELO en rad (ya convertidos desde el
%           encoder con signo y offset, ver simular_cinematica_directa.m):
%             th0 : giro de coxa alrededor del eje vertical de J0 (+ antihorario visto desde arriba)
%             th1 : elevacion del femur respecto a la horizontal (+ hacia arriba)
%             th2 : flexion de la tibia respecto a la prolongacion del femur (+ hacia abajo)
%   g     = estructura de geometria (mm, rad): L1, h1, d, L2, L3, gamma, tramos
%
%   pie   = posicion del pie [x;y;z] en mm en el marco de J0
%   P     = [J0 J1 J2 pie] (3x4) para dibujar
%   tibia = puntos (3xN) de la tibia doblada por tramos, para dibujar
%
%   Ecuaciones (plano de la pata, r = radial, z = vertical):
%       phi = th1 - th2 - gamma
%       r   = L1 + L2*cos(th1) + L3*cos(phi)
%       z   = h1 + L2*sin(th1) + L3*sin(phi)
%   Planta (giro de coxa + desfase lateral d del plano de la pata):
%       x = r*cos(th0) - d*sin(th0)
%       y = r*sin(th0) + d*cos(th0)

    th0 = th(1); th1 = th(2); th2 = th(3);

    % ---- Plano de la pata (r, z) ----
    phi = th1 - th2 - g.gamma;               % direccion absoluta de la recta J2 -> pie
    r1 = g.L1;                      z1 = g.h1;                      % J1
    r2 = r1 + g.L2*cos(th1);        z2 = z1 + g.L2*sin(th1);        % J2
    r3 = r2 + g.L3*cos(phi);        z3 = z2 + g.L3*sin(phi);        % pie

    % ---- Planta: giro de coxa ----
    c = cos(th0); s = sin(th0);
    alPlano = @(r, z) [r*c - g.d*s; r*s + g.d*c; z];

    J0  = [0; 0; 0];
    J1  = alPlano(r1, z1);
    J2  = alPlano(r2, z2);
    pie = alPlano(r3, z3);
    P   = [J0 J1 J2 pie];

    % ---- Tibia por tramos (solo para dibujar y comprobar L3, gamma) ----
    if nargout > 2
        psi = th1 - th2;                     % direccion absoluta del tramo 1
        rr = r2; zz = z2; tibia = zeros(3, size(g.tramos, 1) + 1);
        tibia(:, 1) = J2;
        for k = 1:size(g.tramos, 1)
            ang = psi + deg2rad(g.tramos(k, 2));
            rr = rr + g.tramos(k, 1)*cos(ang);
            zz = zz + g.tramos(k, 1)*sin(ang);
            tibia(:, k + 1) = alPlano(rr, zz);
        end
    end
end
