function diagnostico_u2()
% Ubica los origenes de las articulaciones del Xacro U2 en su pose cero.
    carpeta = fileparts(mfilename('fullpath'));
    urdf = fullfile(carpeta, '..', 'urdf', ...
        'ROBOT_ESAMBLE_description-20261104T154508Z-1-001', ...
        'ROBOT_ESAMBLE_description', 'urdf', 'ROBOT_ESAMBLE.xacro');

    % Este archivo declara los eslabones y juntas sin macros.
    % Las inclusiones aportan materiales, transmisiones y datos de Gazebo.
    descripcion = fileread(urdf);
    descripcion = regexprep(descripcion, '<xacro:include\b[^>]*/>', '');
    % La comparacion de marcos se puede hacer sin resolver rutas a mallas.
    descripcion = regexprep(descripcion, ...
        '(?s)<(visual|collision)>.*?</\1>', '');
    robot = importrobot(descripcion, 'urdf', 'DataFormat', 'struct');
    q = homeConfiguration(robot);
    assert(numel(q) == 18, 'Se esperaban 18 articulaciones moviles.');
    [q.JointPosition] = deal(0);

    segmentos = {'coxa', 'femur', 'tibia'};
    fprintf('Origenes articulares en %s, q = 0 [mm]\n', robot.BaseName);
    for pata = 1:6
        for junta = 1:3
            cuerpo = sprintf('pata_%d_%s_link_1', pata, segmentos{junta});
            transformacion = getTransform(robot, q, cuerpo);
            xyz = 1000 * transformacion(1:3, 4);
            fprintf('Pata %d  J%d  %9.3f  %9.3f  %9.3f\n', ...
                pata, junta - 1, xyz(1), xyz(2), xyz(3));
        end
    end
end
