function [pie, puntos] = cinematica_pata1_u2(q, puntoCero)
% q = [J0 J1 J2] en radianes con la convencion de mando de Fusion.
% pie es un vector columna en mm en el marco raiz del ensamble de Fusion.
% puntos contiene las columnas J0, J1, J2 y pie en el mismo marco.
% puntoCero permite seguir otro punto fijo de la tibia, dado en mm en q = 0.
    validateattributes(q, {'numeric'}, {'real', 'finite', 'vector', 'numel', 3});
    carpeta = fileparts(mfilename('fullpath'));
    datos = jsondecode(fileread(fullfile(carpeta, 'medicion_fusion_u2.json')));
    j0 = buscarUnion(datos.uniones, 'base_J0:1', 'coxa_link_1:1');
    j1 = buscarUnion(datos.uniones, 'coxa_link_1:1', 'femur_link_1:1');
    j2 = buscarUnion(datos.uniones, 'femur_link_1:1', 'tibia_link_1:1');
    assert(max(abs([j0.angulo_rad, j1.angulo_rad, j2.angulo_rad])) < 1e-8, ...
        'La medicion de referencia de Fusion debe estar en la pose cero.');

    p0 = j0.origen_mm(:);
    p1 = j1.origen_mm(:);
    p2 = j2.origen_mm(:);
    if nargin < 2
        cuerpos = datos.tibias.cuerpos;
        suela = cuerpos(strcmp({cuerpos.nombre}, 'soporte_tibia_A'));
        puntoCero = suela.min_z_mm;
    end
    validateattributes(puntoCero, {'numeric'}, {'real', 'finite', 'vector', 'numel', 3});

    % Los signos reproducen los giros positivos observados en Fusion.
    a0 = j0.eje(:);
    a1 = -j1.eje(:);
    a2 = j2.eje(:);
    coxa = p1 - p0;
    femur = p2 - p1;
    tibia = puntoCero(:) - p2;

    J1 = p0 + girar(coxa, a0, q(1));
    J2 = J1 + girar(girar(femur, a1, q(2)), a0, q(1));
    pie = J1 + girar(girar(femur + girar(tibia, a2, q(3)), a1, q(2)), a0, q(1));
    puntos = [p0, J1, J2, pie];
end

function union = buscarUnion(uniones, padre, hijo)
    coincide = arrayfun(@(u) ~u.suprimida && ...
        any(strcmp(u.ocurrencias, padre)) && any(strcmp(u.ocurrencias, hijo)), uniones);
    assert(nnz(coincide) == 1, 'Se esperaba una union activa entre %s y %s.', padre, hijo);
    union = uniones(coincide);
end

function resultado = girar(vector, eje, angulo)
    eje = eje / norm(eje);
    paralelo = eje * dot(eje, vector);
    perpendicular = vector - paralelo;
    resultado = paralelo + cos(angulo) * perpendicular ...
        + sin(angulo) * cross(eje, perpendicular);
end
