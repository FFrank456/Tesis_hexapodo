function simular_pata1_u2()
% Simulacion interactiva de la pata 1 de la Unidad 2.
% Los controles exploran giros cinematicos sin evaluar colisiones ni topes.
    [~, referencia] = cinematica_pata1_u2([0 0 0]);
    origen = referencia(:, 1);
    tinta = [0.12 0.19 0.26];
    % Mostrar el montaje y el movimiento en coordenadas del ensamble.
    figura = figure('Name', 'Unidad 2 | Cinemática de la pata 1', ...
        'NumberTitle', 'off', 'Color', 'white', 'Position', [120 100 1000 625]);
    vista = axes('Parent', figura, 'Units', 'normalized', ...
        'Position', [0.08 0.15 0.62 0.76], 'Color', [0.97 0.98 0.99], ...
        'XColor', tinta, 'YColor', tinta, 'ZColor', tinta, 'GridColor', tinta);
    hold(vista, 'on');
    grid(vista, 'on');
    box(vista, 'on');
    xlabel(vista, 'X del ensamble (mm)');
    ylabel(vista, 'Y del ensamble (mm)');
    zlabel(vista, 'Z del ensamble (mm)');
    title(vista, 'Pata 1 de la Unidad 2', 'Color', tinta);
    axis(vista, 'equal');
    view(vista, [-59 26]);
    xlim(vista, origen(1) + [-20 225]);
    ylim(vista, origen(2) + [-225 25]);
    zlim(vista, origen(3) + [-145 120]);

    cero = plot3(vista, referencia(1,:), referencia(2,:), referencia(3,:), ...
        '--', 'Color', [0.54 0.59 0.65], 'LineWidth', 1.6);
    pata = plot3(vista, nan, nan, nan, '-o', 'Color', [0.04 0.32 0.55], ...
        'LineWidth', 3, 'MarkerFaceColor', 'white', 'MarkerSize', 7);
    apoyo = plot3(vista, nan, nan, nan, 'o', 'MarkerFaceColor', [0.85 0.29 0.13], ...
        'MarkerEdgeColor', 'white', 'MarkerSize', 10);
    hLeyenda = legend(vista, [cero pata apoyo], ...
        {'Pose cero', 'Pata simulada', 'Punto del pie'}, ...
        'Location', 'southoutside', 'Orientation', 'horizontal');
    set(hLeyenda, 'AutoUpdate', 'off', 'Color', 'white', 'TextColor', tinta);

    nombres = {'J0 · Coxa', 'J1 · Fémur', 'J2 · Tibia'};
    etiquetas = gobjects(1,3);
    controles = gobjects(1,3);
    for j = 1:3
        alto = 0.82 - (j-1)*0.14;
        etiquetas(j) = uicontrol(figura, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.72 alto 0.24 0.04], 'HorizontalAlignment', 'left', ...
            'BackgroundColor', 'white', 'ForegroundColor', tinta, 'FontSize', 12);
        controles(j) = uicontrol(figura, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.72 alto-0.04 0.24 0.035], 'Min', -30, 'Max', 30, ...
            'Value', 0, 'SliderStep', [1 5]/60, 'Callback', @(~,~) actualizar());
    end

    xyz = uicontrol(figura, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.72 0.245 0.25 0.17], 'HorizontalAlignment', 'left', ...
        'BackgroundColor', 'white', 'ForegroundColor', tinta, 'FontSize', 12);
    uicontrol(figura, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.72 0.17 0.24 0.062], 'String', 'Pose combinada', ...
        'FontSize', 11, 'Callback', @(~,~) irAPose([10 20 15]));
    uicontrol(figura, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.72 0.095 0.24 0.062], 'String', 'Volver a cero', ...
        'FontSize', 11, 'Callback', @(~,~) irAPose([0 0 0]));
    actualizar();

    function irAPose(angulos)
        set(controles, {'Value'}, num2cell(angulos(:)));
        actualizar();
    end

    function actualizar()
        grados = [controles.Value];
        [pie, puntos] = cinematica_pata1_u2(deg2rad(grados));
        set(pata, 'XData', puntos(1,:), 'YData', puntos(2,:), 'ZData', puntos(3,:));
        set(apoyo, 'XData', pie(1), 'YData', pie(2), 'ZData', pie(3));
        for k = 1:3
            etiquetas(k).String = sprintf('%s    %+.1f°', nombres{k}, grados(k));
        end
        xyz.String = {'Punto del pie en Fusion', ...
            sprintf('X  %+.1f mm', pie(1)), ...
            sprintf('Y  %+.1f mm', pie(2)), ...
            sprintf('Z  %+.1f mm', pie(3))};
    end
end
