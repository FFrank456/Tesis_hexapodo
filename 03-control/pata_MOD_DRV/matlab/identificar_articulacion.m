function P = identificar_articulacion(archivos, motor, varargin)
%IDENTIFICAR_ARTICULACION Fricción, inercia y gravedad de una articulación de la
%pata a partir de los CSV del dashboard, y ganancias sugeridas para el firmware.
%
%   P = identificar_articulacion('ensayo.csv', 2)
%   P = identificar_articulacion({'rampa.csv','vaivén.csv','escalones.csv'}, 3, ...
%                                'fc_vel_hz', 3, 'gravedad', true)
%
%   motor: 1 = Coxa, 2 = Fémur, 3 = Tibia (columnas m1_, m2_, m3_ del CSV).
%
%   Modelo identificado (unidades del firmware: A equivalentes del CS, rad, rad/s):
%
%       I = Jeq*alfa + Ic*sign(w) + b*w + G*cos(theta + theta0)
%
%       I      corriente con signo = sign(PWM) * corriente medida      [A eq]
%       alfa   aceleración del eje de salida                           [rad/s^2]
%       Jeq    inercia equivalente (inercia / constante de torque)     [A*s^2/rad]
%       Ic     fricción de Coulomb                                     [A eq]
%       b      fricción viscosa                                        [A*s/rad]
%       G, theta0  gravedad, igual que el comando GRAV del firmware    [A eq, rad]
%
%   Además, con los tramos en lazo abierto (pestaña PWM / comando SEQ):
%     - PWM de arranque (fricción estática) en cada sentido -> FRIC
%     - modelo PWM -> velocidad  tau*dw/dt = K*(u - u0*sign(w)) - w (+ gravedad)
%
%   Opciones (pares nombre/valor):
%     'fc_vel_hz'     ancho de banda del lazo de velocidad [Hz]        (3)
%     'razon_pos'     ancho de banda velocidad / posición               (4)
%     'gravedad'      true/false: incluir el término de gravedad (true; false para la coxa
%                     si gira en un plano horizontal)
%     'w_min'         |w| mínima para usar una muestra [rad/s]          (0.3)
%     'suavizado'     muestras de la media móvil para la aceleración    (5)
%     'retardo_vel_s' retardo de la velocidad del firmware a compensar [s] (0.010:
%                     mitad de la ventana de 20 ms de ENCODER_SPEED_MIN_WINDOW_US)
%     'graficar'      true/false                                         (true)
%     'guardar_png'   nombre de archivo para guardar la figura          ('')
%
%   Requisitos de los ensayos (ver README.md de esta carpeta):
%     - el CERO del encoder debe ser el mismo en todos los CSV de una articulación
%       (ponerlo con el eslabón horizontal: así theta0 ~ 0);
%     - registrar con el botón Grabar del dashboard (TEL,MULTI, 100 Hz).
%
%   Funciona en MATLAB y en GNU Octave, sin toolboxes.

    op = struct('fc_vel_hz', 3, 'razon_pos', 4, 'gravedad', true, 'w_min', 0.3, ...
                'suavizado', 5, 'retardo_vel_s', 0.010, 'graficar', true, 'guardar_png', '');
    for k = 1:2:numel(varargin)
        if ~isfield(op, varargin{k})
            error('identificar_articulacion:opcion', 'Opción desconocida: %s', varargin{k});
        end
        op.(varargin{k}) = varargin{k + 1};
    end
    if ischar(archivos), archivos = {archivos}; end
    nombres = {'Coxa', 'Fémur', 'Tibia'};

    % ------------------------------------------------------------------ datos
    S = struct('t', [], 'u', [], 'I', [], 'th', [], 'w', [], 'a', [], 'mode', [], ...
               'ok', [], 'brake', [], 'file', []);
    arranques = struct('pwm', [], 'signo', [], 'theta', [], 'corriente', [], 't', [], 'file', []);
    for f = 1:numel(archivos)
        D = leer_csv_pata(archivos{f});
        m = D.m(motor);
        t = D.t;
        w = m.spd;                                   % rad/s, NaN mientras no es válida
        th = m.pos;
        ws = suavizar(w, op.suavizado);
        % La velocidad del firmware es una diferencia en una ventana de 20 ms: está
        % retrasada ~10 ms respecto a la corriente. Se adelanta para alinearlas.
        n_ret = round(op.retardo_vel_s / max(median(diff(t)), eps));
        if n_ret > 0
            ws = [ws(1 + n_ret:end); nan(n_ret, 1)];
        end
        a = suavizar(gradiente(ws, t), op.suavizado);
        I = sign(m.duty) .* m.cur;                   % el CS mide magnitud
        estado_ok = ismember(m.status, [1 9 11 13 14 15]);
        frenando = bitand(uint32(m.flags), uint32(16)) > 0;
        ok = estado_ok & isfinite(w) & isfinite(th) & isfinite(a) & isfinite(I) & ~frenando;

        S.t = [S.t; t + 1e6 * f];                    % separa archivos en el tiempo
        S.u = [S.u; m.duty]; S.I = [S.I; I]; S.th = [S.th; th]; S.w = [S.w; ws];
        S.a = [S.a; a]; S.mode = [S.mode; m.mode]; S.ok = [S.ok; ok];
        S.brake = [S.brake; frenando]; S.file = [S.file; f * ones(size(t))];

        % Fricción estática: en lazo abierto, el eje estaba quieto >= 0.3 s y arranca.
        quieto = abs(ws) < 0.15;
        abierto = m.mode == 0;
        dt = median(diff(t));
        nq = max(1, round(0.3 / dt));
        n1 = round(1.0 / dt);
        k = nq + 1;
        while k <= numel(t)
            % Inicio del movimiento: deja de estar quieto tras >= 0.3 s quieto, y en
            % menos de 1 s supera 0.4 rad/s o avanza 0.015 rad en el mismo sentido.
            if abierto(k) && ~quieto(k) && isfinite(ws(k)) && all(quieto(k - nq:k - 1)) && m.duty(k) ~= 0
                kf = min(numel(t), k + n1);
                % (el ensayo automático del dashboard corta apenas se mueve 0.1 rad)
                confirma = any(abs(ws(k:kf)) > 0.4 & sign(ws(k:kf)) == sign(ws(k))) || ...
                           any(sign(ws(k)) * (th(k:kf) - th(k - 1)) > 0.015);
            else
                confirma = false;
            end
            if confirma
                % PWM aplicado justo antes del arranque (el escalón que lo provocó)
                j0 = max(1, k - round(0.05 / dt));
                [arranques.pwm(end + 1), jm] = max(abs(m.duty(j0:k)));
                % Corriente con signo en ese instante (eje aún casi quieto: sin fuerza
                % contraelectromotriz, I es proporcional al PWM).
                arranques.corriente(end + 1) = I(j0 + jm - 1);
                arranques.signo(end + 1) = sign(m.duty(k));
                arranques.theta(end + 1) = th(k);
                arranques.t(end + 1) = t(k);
                arranques.file(end + 1) = f;
                k = k + nq;                           % no contar el mismo arranque dos veces
            else
                k = k + 1;
            end
        end
    end
    S.ok = logical(S.ok);
    S.brake = logical(S.brake);
    P.motor = motor;
    P.nombre = nombres{motor};
    P.archivos = archivos;
    P.opciones = op;
    P.arranque = arranques;

    % ------------------------------------------- modelo en corriente (todos los modos)
    usa = S.ok & abs(S.w) > op.w_min;
    if op.gravedad
        X = @(i) [S.a(i), sign(S.w(i)), S.w(i), cos(S.th(i)), sin(S.th(i))];
        etiquetas = {'Jeq', 'Ic', 'b', 'cos', 'sin'};
    else
        X = @(i) [S.a(i), sign(S.w(i)), S.w(i)];
        etiquetas = {'Jeq', 'Ic', 'b'};
    end
    idx = find(usa);
    P.corriente = struct('valido', false);
    if numel(idx) < 50
        warning('identificar_articulacion:pocos', ...
                'Solo %d muestras con |w| > %.2f rad/s: no se identifica el modelo en corriente.', ...
                numel(idx), op.w_min);
    else
        [c, idx, r2, cond_x] = minimos_cuadrados_robustos(X(idx), S.I(idx), idx);
        C.valido = true;
        C.Jeq = c(1); C.Ic = c(2); C.b = c(3);
        if op.gravedad
            C.G = hypot(c(4), c(5));
            C.theta0 = atan2(-c(5), c(4));
            if C.G < 0.01, C.theta0 = 0; end
        else
            C.G = 0; C.theta0 = 0;
        end
        C.r2 = r2;
        C.n = numel(idx);
        C.condicion = cond_x;
        C.idx = idx;
        C.coef = c;
        C.etiquetas = etiquetas;
        C.sentidos = [sum(S.w(idx) > 0), sum(S.w(idx) < 0)];
        P.corriente = C;
    end

    % --------------------------------------------- modelo PWM -> velocidad (lazo abierto)
    abiertas = find(S.ok & S.mode == 0 & abs(S.w) > op.w_min & S.u ~= 0);
    P.pwm = struct('valido', false);
    if numel(abiertas) >= 50
        if op.gravedad
            Xu = [S.u(abiertas), S.w(abiertas), sign(S.w(abiertas)), cos(S.th(abiertas)), sin(S.th(abiertas))];
        else
            Xu = [S.u(abiertas), S.w(abiertas), sign(S.w(abiertas))];
        end
        [cu, abiertas, r2u] = minimos_cuadrados_robustos(Xu, S.a(abiertas), abiertas);
        % alfa = (K/tau) u - (1/tau) w - (K u0/tau) sign(w) + ...
        Wm.tau = -1 / cu(2);
        Wm.K = cu(1) * Wm.tau;
        Wm.u0 = -cu(3) / cu(1);
        Wm.r2 = r2u;
        Wm.n = numel(abiertas);
        Wm.valido = isfinite(Wm.tau) && Wm.tau > 0 && Wm.K > 0;
        Wm.idx = abiertas;
        P.pwm = Wm;
    end

    % ----------------------------------------------------------------- ganancias
    P.sugerido = ganancias(P, op, S);
    imprimir(P);
    if op.graficar
        graficar(P, S, op);
    end
end

% =============================================================================
function G = ganancias(P, op, S)
    m = sprintf('M%d', P.motor);
    G = struct();
    wc = 2 * pi * op.fc_vel_hz;
    G.comandos = {};
    C = P.corriente;
    if C.valido && C.Jeq > 0
        % Planta corriente -> velocidad: w/I = 1/(Jeq s + b). El cero del PI cancela
        % el polo (Ki/Kp = b/Jeq) y Kp fija el ancho de banda: wc = Kp/Jeq.
        G.spi_kp = C.Jeq * wc;
        if C.b > 0
            G.spi_ki = C.b * wc;
        else
            G.spi_ki = G.spi_kp * wc / 5;        % b mal identificado: cero en wc/5
        end
        G.ppi_kp = wc / op.razon_pos;
        G.ppi_ki = 0;
        G.comandos{end + 1} = sprintf('%s,SPI,%.5f,%.5f', m, G.spi_kp, G.spi_ki);
        G.comandos{end + 1} = sprintf('%s,PPI,%.3f,0', m, G.ppi_kp);
        % Límite de corriente: lo que pidió el ensayo con margen, sin pasar de 0.75 A
        % (bloqueo de los Pololu HPCB 12 V) ni bajar de 0.3 A.
        % Solo lazo cerrado: los escalones en lazo abierto piden más de lo normal.
        cerrado = S.ok & S.mode > 0;
        if sum(cerrado) < 50, cerrado = S.ok; end
        I_abs = sort(abs(S.I(cerrado)));
        p99 = I_abs(max(1, round(0.99 * numel(I_abs))));
        G.ilimit = min(0.75, max(0.3, 1.3 * p99));
        G.comandos{end + 1} = sprintf('%s,ILIMIT,%.3f', m, G.ilimit);
        % Aceleración que deja la mitad de la corriente libre para corregir.
        libre = G.ilimit - abs(C.Ic) - abs(C.G);
        if libre > 0
            G.accel = min(40, 0.5 * libre / C.Jeq);
            G.vmax = min(32, 0.8 * (G.ilimit - abs(C.Ic) - abs(C.G)) / max(C.b, 1e-4));
            G.vmax = max(0.5, min(G.vmax, 10));
            G.comandos{end + 1} = sprintf('%s,PLIMIT,%.2f,%.1f,0.02', m, G.vmax, G.accel);
        end
        if C.G >= 0.01
            G.comandos{end + 1} = sprintf('%s,GRAV,%.4f,%.4f', m, C.G, C.theta0);
        else
            G.comandos{end + 1} = sprintf('%s,GRAV,0,0', m);
        end
    end
    % Fricción estática: 70 % del PWM de arranque. Si hay gravedad identificada, se
    % descuenta del arranque la parte que la gravedad ayudó o estorbó:
    %   Is = |I_arranque - G*cos(theta+theta0)|  y  PWM_fric = Is * (PWM/I) del arranque.
    A = P.arranque;
    G.fric = NaN;
    if ~isempty(A.pwm)
        pwm_fric = A.pwm;
        if C.valido && C.G >= 0.01
            grav = C.G * cos(A.theta + C.theta0);
            Is = abs(A.corriente - grav);
            escala = A.pwm ./ max(abs(A.corriente), 1e-3);
            corr = isfinite(Is) & abs(A.corriente) > 0.005;
            pwm_fric(corr) = Is(corr) .* escala(corr);
            G.is_a = median(Is(corr));
        end
        G.pwm_arranque_corregido = pwm_fric;
        pos = pwm_fric(A.signo > 0); neg = pwm_fric(A.signo < 0);
        candidatos = [];
        if ~isempty(pos), candidatos(end + 1) = median(pos); end
        if ~isempty(neg), candidatos(end + 1) = median(neg); end
        G.fric = 0.7 * min(candidatos);
    elseif P.pwm.valido && P.pwm.u0 > 0
        G.fric = 0.7 * P.pwm.u0;
    end
    if ~isfinite(G.fric) && C.valido && C.Ic > 0
        % Sin rampa: Ic (A) a PWM con la relación PWM/I medida a baja velocidad.
        j = S.ok & abs(S.I) > 0.02 & abs(S.w) < 2 & S.u ~= 0;
        if sum(j) > 20
            G.fric = 0.7 * C.Ic * median(abs(S.u(j)) ./ abs(S.I(j)));
            G.fric_desde_Ic = true;
        end
    end
    if isfinite(G.fric)
        G.fric = min(70, G.fric);
        G.comandos{end + 1} = sprintf('%s,FRIC,%.1f', m, G.fric);
    end
end

function imprimir(P)
    fprintf('\n=== Identificación M%d %s ===\n', P.motor, P.nombre);
    fprintf('Archivos: %s\n', strjoin(P.archivos, ', '));
    A = P.arranque;
    if ~isempty(A.pwm)
        fprintf('\nFricción estática (PWM de arranque, lazo abierto):\n');
        pos = A.pwm(A.signo > 0); neg = A.pwm(A.signo < 0);
        if ~isempty(pos), fprintf('  sentido + : %5.1f %%  (mediana de %d arranques)\n', median(pos), numel(pos)); end
        if ~isempty(neg), fprintf('  sentido - : %5.1f %%  (mediana de %d arranques)\n', median(neg), numel(neg)); end
        if isfield(P.sugerido, 'is_a')
            pc = P.sugerido.pwm_arranque_corregido;
            fprintf('  sin el efecto de la gravedad: Is = %.3f A, PWM %.1f %% (+) / %.1f %% (-)\n', ...
                    P.sugerido.is_a, median_o_nan(pc(A.signo > 0)), median_o_nan(pc(A.signo < 0)));
        end
    else
        fprintf('\nFricción estática: no hay arranques en lazo abierto (ensayo 1).\n');
    end
    C = P.corriente;
    if C.valido
        fprintf('\nModelo en corriente  I = Jeq*alfa + Ic*sign(w) + b*w + G*cos(theta+theta0)\n');
        fprintf('  Jeq    = %.5f A*s^2/rad   (inercia / constante de torque)\n', C.Jeq);
        fprintf('  Ic     = %.4f A           (Coulomb)\n', C.Ic);
        fprintf('  b      = %.5f A*s/rad     (viscosa)\n', C.b);
        fprintf('  G      = %.4f A, theta0 = %.4f rad (%.1f°)\n', C.G, C.theta0, C.theta0 * 180 / pi);
        fprintf('  R^2    = %.3f con %d muestras (%d en sentido +, %d en sentido -)\n', ...
                C.r2, C.n, C.sentidos(1), C.sentidos(2));
        if C.Jeq <= 0
            fprintf('  AVISO: Jeq <= 0. Faltan aceleraciones en el ensayo (usa el ensayo 2 con varias Acel.).\n');
        end
        if any(C.sentidos < 100)
            fprintf('  AVISO: pocos datos en un sentido; la fricción y la gravedad se confunden.\n');
        end
        if C.condicion > 1e3
            fprintf('  AVISO: regresores casi colineales (cond = %.0f): mueve la articulación en un rango\n', C.condicion);
            fprintf('         de ángulos más amplio y en ambos sentidos.\n');
        end
        if C.r2 < 0.6
            fprintf('  AVISO: R^2 bajo. Revisa que el cero del encoder sea el mismo en todos los CSV\n');
            fprintf('         y que la pata no haya tocado el suelo o un tope.\n');
        end
    end
    W = P.pwm;
    if W.valido
        fprintf('\nModelo PWM -> velocidad (lazo abierto)  tau*dw/dt = K*(u - u0*sign(w)) - w\n');
        fprintf('  K = %.4f (rad/s)/%%   tau = %.3f s   u0 = %.1f %%   R^2 = %.3f (%d muestras)\n', ...
                W.K, W.tau, W.u0, W.r2, W.n);
    end
    G = P.sugerido;
    if ~isempty(G.comandos)
        fprintf('\nComandos sugeridos (cópialos en el cuadro de comando manual del dashboard,\n');
        fprintf('con el motor en STOP; Límite CS y Techo PWM también se ponen en sus campos):\n');
        for k = 1:numel(G.comandos), fprintf('  %s\n', G.comandos{k}); end
        if isfield(G, 'spi_kp')
            fprintf('  (lazo de velocidad a %.1f Hz, posición a %.2f Hz)\n', ...
                    P.opciones.fc_vel_hz, P.opciones.fc_vel_hz / P.opciones.razon_pos);
        end
    end
    fprintf('\n');
end

function graficar(P, S, op)
    C = P.corriente;
    fig = figure('Name', sprintf('Identificación M%d %s', P.motor, P.nombre), 'Color', 'w');
    if C.valido
        i = C.idx;
        c = C.coef(:);
        if op.gravedad
            Xi = [S.a(i), sign(S.w(i)), S.w(i), cos(S.th(i)), sin(S.th(i))];
        else
            Xi = [S.a(i), sign(S.w(i)), S.w(i)];
        end
        pred = Xi * c;

        subplot(2, 2, 1);
        plot(1:numel(i), S.I(i), '.', 'MarkerSize', 4); hold on;
        plot(1:numel(i), pred, '-', 'LineWidth', 1);
        title({'Corriente: medida vs modelo', sprintf('R^2 = %.3f', C.r2)});
        xlabel('muestra usada'); ylabel('I [A eq]'); legend('medida', 'modelo'); grid on;

        subplot(2, 2, 2);  % fricción: quitar inercia y gravedad
        fric = S.I(i) - c(1) * S.a(i);
        if op.gravedad, fric = fric - c(4) * cos(S.th(i)) - c(5) * sin(S.th(i)); end
        plot(S.w(i), fric, '.', 'MarkerSize', 4); hold on;
        wl = linspace(min(S.w(i)), max(S.w(i)), 200);
        plot(wl, C.Ic * sign(wl) + C.b * wl, 'r-', 'LineWidth', 1.5);
        title({'Fricción', sprintf('Ic = %.3f A, b = %.4f A s/rad', C.Ic, C.b)});
        xlabel('w [rad/s]'); ylabel('I sin inercia ni gravedad [A]'); grid on;

        subplot(2, 2, 3);  % gravedad: quitar inercia y fricción
        if op.gravedad
            grav = S.I(i) - c(1) * S.a(i) - c(2) * sign(S.w(i)) - c(3) * S.w(i);
            plot(S.th(i), grav, '.', 'MarkerSize', 4); hold on;
            tl = linspace(min(S.th(i)), max(S.th(i)), 200);
            plot(tl, C.G * cos(tl + C.theta0), 'r-', 'LineWidth', 1.5);
            title({'Gravedad', sprintf('G = %.3f A, \\theta_0 = %.2f rad', C.G, C.theta0)});
            xlabel('\theta [rad]'); ylabel('I sin inercia ni fricción [A]'); grid on;
        else
            axis off; text(0.1, 0.5, 'Gravedad no incluida (opción ''gravedad'', false)');
        end
    end
    subplot(2, 2, 4);
    A = P.arranque;
    if ~isempty(A.pwm)
        stem(find(A.signo > 0), A.pwm(A.signo > 0), 'b'); hold on;
        stem(find(A.signo < 0), A.pwm(A.signo < 0), 'r');
        title({'Fricción estática', 'PWM de arranque'}); xlabel('evento'); ylabel('|PWM| [%]'); grid on;
        legend('sentido +', 'sentido -');
    elseif P.pwm.valido
        j = P.pwm.idx;
        plot(S.u(j), S.w(j), '.', 'MarkerSize', 4);
        title(sprintf('Lazo abierto: K = %.3f, \\tau = %.3f s', P.pwm.K, P.pwm.tau));
        xlabel('PWM [%]'); ylabel('w [rad/s]'); grid on;
    else
        axis off; text(0.1, 0.5, 'Sin datos en lazo abierto');
    end
    if ~isempty(op.guardar_png)
        print(fig, op.guardar_png, '-dpng', '-r120');
    end
end

% =============================================================================
function [c, idx, r2, cond_x] = minimos_cuadrados_robustos(X, y, idx)
%Mínimos cuadrados con una pasada de rechazo de atípicos (> 3 sigma).
    c = X \ y;
    r = y - X * c;
    s = 1.4826 * median(abs(r - median(r)));
    if s > 0
        bueno = abs(r) < 3 * s;
        if sum(bueno) > size(X, 2) * 10
            X = X(bueno, :); y = y(bueno); idx = idx(bueno);
            c = X \ y;
            r = y - X * c;
        end
    end
    r2 = 1 - sum(r .^ 2) / max(sum((y - mean(y)) .^ 2), eps);
    Xn = X ./ max(sqrt(sum(X .^ 2, 1)), eps);
    cond_x = cond(Xn);
end

function y = suavizar(x, k)
%Media móvil centrada que tolera NaN (los huecos se interpolan solo para el cálculo).
    x = x(:);
    v = isfinite(x);
    y = nan(size(x));
    if sum(v) < 2, return; end
    n = (1:numel(x))';
    xi = interp1(n(v), x(v), n, 'linear', 'extrap');
    k = max(1, round(k));
    ker = ones(k, 1) / k;
    ys = conv(xi, ker, 'same');
    % Bordes: corrige el peso del kernel truncado.
    w = conv(ones(size(xi)), ker, 'same');
    y = ys ./ w;
    y(~v) = NaN;
end

function d = gradiente(x, t)
    d = nan(size(x));
    v = isfinite(x);
    if sum(v) < 3, return; end
    d = gradient(x, t);
end

function m = median_o_nan(x)
    if isempty(x), m = NaN; else, m = median(x); end
end
