function [th, ok, info] = ci_pata(pie, g, rama)
% CI_PATA  Cinematica inversa analitica de una pata de 3 GDL (coxa-femur-tibia).
%
%   [th, ok, info] = ci_pata(pie, g)
%   [th, ok, info] = ci_pata(pie, g, rama)
%
%   pie  = posiciones del pie [x;y;z] en mm en el marco de J0 (3xN, una columna por punto)
%   g    = geometria (la misma estructura que usa cd_pata: L1, h1, d, L2, L3, gamma)
%   rama = +1 (por defecto): rodilla normal, la tibia doblada hacia abajo respecto
%                            de la recta del femur (th2 + gamma >= 0)
%          -1              : la otra solucion (tibia doblada hacia arriba)
%
%   th   = angulos DEL MODELO [th0; th1; th2] en rad (3xN), con las mismas
%          convenciones que cd_pata. Para pasarlos a encoder: q = (th - enc.offset) ./ enc.signo
%   ok   = 1xN logico: el punto es alcanzable (sin mirar limites de las juntas)
%   info = .c      coseno del angulo de la rodilla (|c| <= 1 si se alcanza)
%          .r      distancia radial en el plano de la pata (mm; < 0 si el pie
%                  queda detras del eje de la coxa)
%          .D      distancia J1 -> pie en el plano de la pata (mm)
%
%   Inversion de las ecuaciones de cd_pata:
%     Planta: (x, y) = Rot(th0) * (r, d)  ->  r = +-sqrt(x^2 + y^2 - d^2)
%                                             th0 = atan2(y, x) - atan2(d, r)
%             (signo de r: el que deja |th0| <= 90 deg)
%     Plano:  a = r - L1,  b = z - h1,  D^2 = a^2 + b^2
%             beta = phi - th1 = -(th2 + gamma)      (angulo de la rodilla)
%             cos(beta) = (D^2 - L2^2 - L3^2) / (2 L2 L3)
%             beta = -rama * acos(cos(beta))
%             th1 = atan2(b, a) - atan2(L3 sin(beta), L2 + L3 cos(beta))
%             th2 = -beta - gamma
%
%   Funciona en MATLAB y en GNU Octave, sin toolboxes.

    if nargin < 3 || isempty(rama), rama = 1; end
    x = pie(1, :); y = pie(2, :); z = pie(3, :);

    % ---- Planta: giro de la coxa ----
    % Dos soluciones: r > 0 (pie por delante del eje de la coxa) o r < 0 (pie por
    % detras, con el femur muy abajo y la tibia muy doblada), con la coxa girada ~180 deg.
    % La coxa solo gira unos +-35 deg, asi que se elige la que deja th0 mas cerca de 0.
    rho2 = x.^2 + y.^2;
    r = sqrt(max(rho2 - g.d^2, 0));
    th0 = atan2(y, x) - atan2(g.d, r);
    detras = abs(atan2(sin(th0), cos(th0))) > pi/2;
    r(detras) = -r(detras);
    th0(detras) = atan2(y(detras), x(detras)) - atan2(g.d, r(detras));

    % ---- Plano de la pata: femur + recta equivalente de la tibia ----
    a = r - g.L1;
    b = z - g.h1;
    D2 = a.^2 + b.^2;
    c = (D2 - g.L2^2 - g.L3^2) / (2 * g.L2 * g.L3);
    ok = rho2 >= g.d^2 & abs(c) <= 1 + 1e-12;
    beta = -rama * acos(min(max(c, -1), 1));
    th1 = atan2(b, a) - atan2(g.L3 * sin(beta), g.L2 + g.L3 * cos(beta));
    th2 = -beta - g.gamma;

    % Angulos en (-pi, pi]
    th = [th0; th1; th2];
    th = atan2(sin(th), cos(th));

    info.c = c;
    info.r = r;
    info.D = sqrt(D2);
end
