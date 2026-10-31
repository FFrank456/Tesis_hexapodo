function resultado = revisar_posicion_cascada(Tvel, Cpos)
% Revisa posicion sobre el lazo de velocidad CERRADO y la cinematica 1/s.
% No cambia firmware ni manda ordenes a la planta.
% Uso con modelo actual: revisar_posicion_cascada(Tvel, pid(6.166,3192,0))
% Sin argumentos: reproduce SOLO los modelos historicos del PDF documentado
% en revision_cascada_20260927.md; no carga el workspace ni el SLX actual.
% El PI por defecto es el candidato que indica la segunda captura del usuario;
% esta revision NO resuelve la discrepancia con 6.166*s + 3192/s.

s = tf('s');
if nargin < 1 || isempty(Tvel)
    Gi = (0.001184*s^2 + 1.441*s + 2.325)/(s^2 + 904.4*s + 3996);
    Ci = pid(8.832,80090,0);
    Gw = 110700/(s^2 + 110.1*s + 172.3);
    Cw = pid(0.0284,0.01392,0);
    Ti = feedback(Ci*Gi,1);
    Tvel = feedback(Cw*Ti*Gw,1);
    origen = 'Modelos historicos del PDF; verificar contra MATLAB actual';
else
    origen = 'Tvel proporcionado: debe ser w_ref -> w, en lazo cerrado';
end
if nargin < 2 || isempty(Cpos)
    Cpos = pid(6.166,3192,0);
end
if ~isequal(size(Tvel),[1 1]) || ~isequal(size(Cpos),[1 1])
    error('Se requieren modelos SISO.');
end
if Tvel.Ts ~= 0 || Cpos.Ts ~= 0
    error('Esta revision usa modelos continuos; no mezcla 1/s con modelos discretos.');
end

Gpos = Tvel/s; % Referencia de velocidad -> posicion fisica del mismo eje.
Lpos = Cpos*Gpos;
Tpos = feedback(Lpos,1);
p = pole(Tpos);
estable = isstable(Tpos);
resultado = struct('origen',origen,'controlador',Cpos,'velocidad_cerrada',Tvel, ...
    'planta_posicion',Gpos,'posicion_cerrada',Tpos,'polos',p,'estable',estable);
fprintf('%s\n',origen);
disp('Controlador utilizado:'); disp(Cpos);
disp('Polos de la cascada de posicion:'); disp(p);
fprintf('Estable en este modelo lineal continuo: %d\n',estable);
if ~isproper(Cpos)
    warning('Controlador impropio: contiene derivada ideal. No trasladar directamente al firmware.');
end
figure('Name','Polos de la cascada de posicion'); pzmap(Tpos); grid on;
if estable
    resultado.stepinfo = stepinfo(Tpos);
    disp(resultado.stepinfo);
    figure('Name','Posicion: cascada completa lineal'); step(Tpos); grid on;
else
    warning('Cascada inestable con estos modelos: revisar planta y sintonizacion antes de activar posicion.');
end
fprintf(['No incluye saturacion PWM/CS, logica coast/inversion, limites de velocidad,\n' ...
    'cuantizacion ni discretizacion. Estabilidad aqui no valida la implementacion fisica.\n']);
end
