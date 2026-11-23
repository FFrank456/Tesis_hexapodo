"""Genera Informe_Unidad2_Control_CAN.docx (OOXML directo, sin dependencias).
Las cifras de desempeno, identificacion y lazos se leen de metricas.json."""
import json, os, zipfile
from xml.sax.saxutils import escape
import matplotlib.image as mi
A = os.path.dirname(os.path.abspath(__file__)); FIG = os.path.join(A, "figuras")
M = json.load(open(os.path.join(A, "metricas.json"), encoding="utf-8"))
def n(v, d=2): return f"{v:.{d}f}".replace(".", ",").replace("-", "−")
B, IMG, CNT = [], [], {"Figura": 0, "Tabla": 0, "Ec": 0, "bm": 0}

def runs(t, rpr=""):
    out = []
    for i, p in enumerate(t.split("**")):
        if p: out.append(f'<w:r><w:rPr>{"<w:b/>" if i % 2 else ""}{rpr}</w:rPr><w:t xml:space="preserve">{escape(p)}</w:t></w:r>')
    return "".join(out)
def p(t, st=None, extra=""): B.append(f'<w:p><w:pPr>{f"<w:pStyle w:val={chr(34)}{st}{chr(34)}/>" if st else ""}{extra}</w:pPr>{runs(t)}</w:p>')
def h(k, t): p(t, f"Heading{k}")
def hs(t): p(t, "SinNumero")
def li(*ts):
    for t in ts: p(t, "Vineta")
def fld(ins, txt): return (f'<w:r><w:fldChar w:fldCharType="begin"/></w:r><w:r><w:instrText xml:space="preserve"> {ins} </w:instrText></w:r>'
    f'<w:r><w:fldChar w:fldCharType="separate"/></w:r><w:r><w:t>{txt}</w:t></w:r><w:r><w:fldChar w:fldCharType="end"/></w:r>')
def cap(tipo, t):
    CNT[tipo] += 1
    B.append(f'<w:p><w:pPr><w:pStyle w:val="Caption"/>{"<w:keepNext/>" if tipo == "Tabla" else ""}</w:pPr><w:r><w:t xml:space="preserve">{tipo} </w:t></w:r>{fld("SEQ "+tipo+" \\* ARABIC", CNT[tipo])}{runs(". "+t)}</w:p>')
def fig(f, t, alt=None, w=15.0):
    r = os.path.join(FIG, f); im = mi.imread(r); k = len(IMG)+1; IMG.append(r); cx = int(w*360000); cy = int(cx*im.shape[0]/im.shape[1])
    if cy > 17*360000: cx = int(cx*17*360000/cy); cy = 17*360000
    B.append(f'<w:p><w:pPr><w:pStyle w:val="Imagen"/></w:pPr><w:r><w:drawing><wp:inline distT="0" distB="0" distL="0" distR="0"><wp:extent cx="{cx}" cy="{cy}"/><wp:docPr id="{k}" name="fig{k}" descr="{escape(alt or t, {chr(34): ""})}"/>'
      f'<a:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture"><pic:pic><pic:nvPicPr><pic:cNvPr id="{k}" name="fig{k}"/><pic:cNvPicPr/></pic:nvPicPr><pic:blipFill><a:blip r:embed="rImg{k}"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill>'
      f'<pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr></pic:pic></a:graphicData></a:graphic></wp:inline></w:drawing></w:r></w:p>')
    cap("Figura", t)
def tabla(t, cab, filas, anchos):
    cap("Tabla", t); tot = 9070; s = sum(anchos); W = [int(tot*a/s) for a in anchos]; W[-1] += tot-sum(W)
    def fila(c, cabz, kn=False):
        return (f'<w:tr><w:trPr><w:cantSplit/>{"<w:tblHeader/>" if cabz else ""}</w:trPr>' + "".join(
            f'<w:tc><w:tcPr><w:tcW w:w="{w}" w:type="dxa"/>{"<w:shd w:val=%sclear%s w:color=%sauto%s w:fill=%sE4ECF4%s/>" % (('"',)*6) if cabz else ""}</w:tcPr><w:p><w:pPr><w:pStyle w:val="Celda"/>{"<w:keepNext/>" if kn else ""}</w:pPr>{runs(("**"+x+"**") if cabz else str(x))}</w:p></w:tc>' for x, w in zip(c, W)) + "</w:tr>")
    B.append('<w:tbl><w:tblPr><w:tblW w:w="9070" w:type="dxa"/><w:tblBorders><w:top w:val="single" w:sz="8" w:color="44546A"/><w:bottom w:val="single" w:sz="8" w:color="44546A"/><w:insideH w:val="single" w:sz="4" w:color="C9D1DA"/></w:tblBorders><w:tblLayout w:type="fixed"/><w:tblCellMar><w:left w:w="70" w:type="dxa"/><w:right w:w="70" w:type="dxa"/></w:tblCellMar></w:tblPr><w:tblGrid>'
        + "".join(f'<w:gridCol w:w="{w}"/>' for w in W) + "</w:tblGrid>" + fila(cab, True, True) + "".join(fila(f, False, len(filas) < 10 and i < len(filas)-1) for i, f in enumerate(filas)) + "</w:tbl>")
    B.append('<w:p><w:pPr><w:pStyle w:val="Celda"/></w:pPr></w:p>')
# ecuaciones OMML
def mr(t): return f'<m:r><m:t xml:space="preserve">{escape(t)}</m:t></m:r>'
def sub(a, b): return f"<m:sSub><m:e>{mr(a)}</m:e><m:sub>{mr(b)}</m:sub></m:sSub>"
def sup(a, b): return f"<m:sSup><m:e>{a}</m:e><m:sup>{b}</m:sup></m:sSup>"
def fr(a, b): return f"<m:f><m:num>{a}</m:num><m:den>{b}</m:den></m:f>"
def ec(*x):
    CNT["Ec"] += 1
    B.append(f'<w:p><w:pPr><w:pStyle w:val="Ecuacion"/></w:pPr><w:r><w:tab/></w:r><m:oMath>{"".join(x)}</m:oMath><w:r><w:tab/><w:t>({CNT["Ec"]})</w:t></w:r></w:p>')
def salto(): B.append('<w:p><w:r><w:br w:type="page"/></w:r></w:p>')
def campo_toc(ins): B.append(f'<w:p>{fld(ins, "Actualizar campos con F9")}</w:p>')

c, fc, tc, fa, fb = M["cox"], M["fem_c"], M["tib_c"], M["fem_a"], M["fem_b"]; ID, LZ = M["identificacion"], M["lazos"]
# ---------------------------------------------------------------- portada
p("UNIVERSIDAD DE INGENIERÍA Y TECNOLOGÍA (UTEC)", "Portada"); p("Proyecto Final de Ingeniería Mecatrónica", "Portada"); p("", "Portada"); p("", "Portada")
p("Control en cascada y comunicación CAN de la Unidad 2: hexápodo modular", "Title"); p("Informe técnico de avance", "Portada"); p("", "Portada")
p("Desarrollo de sistema robótico cooperativo inspirado en insectos para el monitoreo de terrenos accidentados", "Portada"); p("", "Portada"); p("", "Portada")
p("Alfredo Erazo Yalopalin", "Portada"); p("Aleskyjubinsl Alcarraz Jaime", "Portada"); p("Flavio Alarcón Muñoz", "Portada"); p("", "Portada"); p("Asesor: Dante Inga Narváez", "Portada"); p("", "Portada"); p("Noviembre de 2026", "Portada"); salto()
hs("Resumen")
p(f"Este informe documenta el avance de la Unidad 2, un hexápodo modular en el que cada pata tiene una placa propia (MOD_DRV) con un ESP32, tres puentes H DRV8876 y tres motores N20 con encoder. Cada placa ejecuta un control en cascada de corriente (1 kHz), velocidad (100 Hz) y posición (50 Hz), y recibe objetivos de una maestra ESP32-S3 por un bus CAN de 500 kbit/s.")
p(f"Se identificó la dinámica de las tres articulaciones (R² entre {n(ID['coxa']['r2'])} y {n(ID['femur_tp']['r2'])}), se diseñaron las ganancias por asignación de polos y se añadieron compensaciones de fricción y gravedad. En las pruebas finales de una pata, el fémur y la tibia completaron 7 de 7 movimientos, con un tiempo medio de {n(fc['t_media'])} s y {n(tc['t_media'])} s, un error final de hasta {n(fc['efin_max']*1000,1)} y {n(tc['efin_max']*1000,1)} mrad y un sobrepaso de hasta {n(max(fc['sob_max'],tc['sob_max']),1)} %. La coxa también completó 7 de 7, pero con un tiempo medio de {n(c['t_media'])} s y un error final de hasta {n(c['efin_max']*1000,1)} mrad.")
p("En la comunicación, 1000 de 1000 mensajes de prueba se respondieron sin pérdidas entre la maestra y una placa, y 7 de 7 órdenes de posición absoluta llegaron con un error de hasta 19 mrad. El soporte para seis patas está programado y compila, pero aún no se ha probado en las placas.")
p("**Palabras clave:** hexápodo modular, control en cascada, bus CAN, compensación de fricción, compensación de gravedad, ESP32.")
salto(); hs("Índice"); campo_toc('TOC \\o "1-2" \\h \\z \\u'); hs("Índice de figuras"); campo_toc('TOC \\h \\z \\c "Figura"'); hs("Índice de tablas"); campo_toc('TOC \\h \\z \\c "Tabla"'); salto()
hs("Términos y abreviaturas")
tabla("Términos usados en el informe", ["Término", "Significado"], [
 ["ACK", "Acuse de recibo: un nodo que recibe bien una trama lo confirma en el mismo mensaje"], ["Arbitraje", "Mecanismo por el que, si dos nodos transmiten a la vez, gana el identificador más bajo sin perder la trama"],
 ["CANH, CANL", "Par diferencial del bus; el estado se lee por la diferencia de tensión entre ambos"], ["CRC", "Código de comprobación que detecta tramas dañadas"], ["CTX, CRX", "Señales lógicas entre el controlador CAN y el transceptor"], ["DLC", "Cantidad de bytes de datos de la trama, de 0 a 8"],
 ["Dominante, recesivo", "Los dos estados del bus: 0 lógico (dominante) y 1 lógico (recesivo, en reposo)"], ["IPROPI", "Salida del DRV8876 proporcional a la corriente del motor"], ["Latido", "Mensaje periódico que indica que la maestra sigue activa"], ["MF", "Margen de fase: distancia a la inestabilidad de un lazo, en grados"],
 ["nFAULT", "Salida del DRV8876 que indica una falla (activa en bajo)"], ["NVS", "Memoria no volátil del ESP32, donde quedan ganancias, pose e ID"], ["PI, P", "Controladores proporcional-integral y proporcional"], ["Sobrepaso", "Cuánto se pasa la posición del objetivo, en % del escalón"],
 ["Stick-slip", "Movimiento a saltos por la diferencia entre fricción estática y dinámica"], ["TEC, REC", "Contadores de errores de transmisión y recepción del controlador CAN"], ["Tiempo de llegada", "Tiempo desde el cambio de objetivo hasta que el control declara HOLD (dentro de tolerancia y quieto 0,1 s)"], ["TWAI", "Nombre que Espressif da a su controlador CAN"]], [2.2, 7])
# ---------------------------------------------------------------- 1
h(1, "Introducción")
p("La Unidad 2 es el hexápodo modular del sistema cooperativo. Su diseño distribuye el control: una maestra decide los movimientos y cada pata cierra sus propios lazos. Así, los lazos rápidos no dependen de la red, y una pata se detiene sola si pierde la comunicación.")
h(2, "Objetivos del avance"); li("Caracterizar la dinámica de las tres articulaciones de una pata.", "Diseñar y validar un control de posición en cascada con compensación de fricción y gravedad.", "Establecer la comunicación CAN entre la maestra y una pata, y medir su desempeño.", "Permitir que la maestra ordene movimientos que la pata ejecute con su propio control.")
h(2, "Alcance"); p("Los resultados corresponden a **una pata** y a pruebas en banco entre el 11 y el 21 de noviembre de 2026. Las pruebas de control y las de CAN usaron placas MOD_DRV distintas del mismo diseño. Los motores probados son N20 de 12 V con reductora 100:1 de un proveedor genérico, sin hoja de datos; por eso no se usan valores de catálogo.")
# ---------------------------------------------------------------- 2
h(1, "Arquitectura del sistema")
p("La Figura 1 muestra la arquitectura prevista para las seis patas. Solo la pata 1 se ha probado.")
fig("lucid1.png", "Arquitectura de comunicación y alimentación de la Unidad 2. Los elementos en línea discontinua están en preparación.")
h(2, "Placa MOD_DRV")
tabla("Componentes principales de la MOD_DRV", ["Función", "Componente", "Detalle"], [["Procesamiento", "ESP32-WROOM-32E", "Control, CAN y puerto serie a 460 800 baudios"], ["Potencia", "3 × DRV8876", "Modo PWM; nFAULT individual por motor"], ["Medida de corriente", "IPROPI con R = 3 kΩ", "3 V/A hacia ADC1; 64 kS/s por canal"], ["Alimentación", "AP63205 y TLV75733P", "V_MOT de 12 V a 5 V y a 3,3 V; también enciende por USB"], ["Comunicación", "Módulo SN65HVD230", "IO21 a CTX e IO22 desde CRX"], ["Encadenado", "P1 y P2", "CANH, CANL y GND en los pines 1, 2 y 3"]], [2, 3, 5])
p("El límite de corriente por hardware lo fijan la referencia VREF y la resistencia de IPROPI [1]:")
ec(sub("I", "TRIP"), mr(" = "), fr(sub("V", "REF"), sub("R", "IPROPI")+mr(" · ")+sub("A", "IPROPI")), mr(" = "), fr(mr("3,3 V"), mr("3 kΩ · 1000 µA/A")), mr(" = 1,1 A"))
tabla("Asignación de pines por articulación", ["Articulación", "IN1 / IN2", "IPROPI", "Encoder A / B", "nFAULT"], [["Coxa (M1)", "IO26 / IO27", "IO32", "IO35 / IO25", "IO5"], ["Fémur (M2)", "IO18 / IO19", "IO33", "IO23 / IO17", "IO14"], ["Tibia (M3)", "IO16 / IO2", "IO34", "IO4 / IO13", "IO15"]], [2, 2, 1.5, 2.3, 1.5])
h(2, "Medida de posición y corriente")
p("El encoder incremental de 7 pulsos por vuelta se lee en cuadratura (×4) antes de la reductora 100:1, lo que da 2800 cuentas por vuelta de salida (2,24 mrad por cuenta). **Esta escala es una suposición sin verificar.** Las corrientes se expresan en amperios equivalentes del sensado: el DRV8876 solo reporta durante el tiempo encendido del PWM, por lo que el valor es la corriente real multiplicada por el ciclo de trabajo.")
# ---------------------------------------------------------------- 3
h(1, "Comunicación CAN")
h(2, "Fundamento")
p("CAN es un bus serie diferencial en el que todos los nodos comparten dos hilos. Cada mensaje lleva un identificador que define su significado y su prioridad. El arbitraje no destructivo, el CRC y el acuse de recibo lo hacen adecuado para control distribuido [2].")
tabla("Parámetros de la comunicación", ["Parámetro", "Valor", "Significado"], [["Velocidad", "500 kbit/s", "Cada bit dura 2 µs"], ["Formato", "CAN 2.0 estándar", "Identificador de 11 bits, hasta 8 bytes"], ["Transceptor", "SN65HVD230, 3,3 V", "Convierte CTX/CRX en CANH/CANL [3]"], ["Terminación", "120 Ω en cada extremo", "≈ 60 Ω esperados entre CANH y CANL; medición pendiente"], ["Latido", "100 ms", "La maestra consulta el estado de cada pata armada"], ["Tiempo límite", "500 ms", "Sin órdenes en ese tiempo, la pata se desarma"]], [2, 2.6, 5])
fig("lucid5.png", "Trama CAN estándar y contenido de la orden IR_A. SOF: inicio; RTR: petición remota; EOF: fin de trama.")
p("Sin bits de relleno, una trama estándar con DLC bytes ocupa, incluido el espacio entre tramas:")
ec(sub("n", "bits"), mr(" = 47 + 8 · DLC ,   "), mr("t = "), fr(sub("n", "bits"), mr("R")))
p("donde R es la velocidad en bit/s. El relleno añade como máximo ⌊(34 + 8·DLC − 1)/4⌋ bits [4].")
tabla("Duración calculada de las tramas a 500 kbit/s", ["DLC", "Uso", "Bits", "Duración (µs)", "Máximo con relleno (µs)"], [[k, u, M["can_bits"][k]["bits"], M["can_bits"][k]["us"], M["can_bits"][k]["us_max"]] for k, u in (("0", "PING, PARO"), ("1", "ARMAR, ESTADO, PONG"), ("2", "CERO, RECHAZADA"), ("7", "Respuesta IDENT"), ("8", "IR_A, respuesta ESTADO"))], [1, 3.5, 1.2, 2, 2.6])
h(2, "Protocolo de aplicación")
tabla("Identificadores", ["ID", "Sentido", "Contenido"], [["0x0FF", "Maestra a todas", "PARO general (máxima prioridad por ser el ID más bajo)"], ["0x100 + pata", "Maestra a pata", "Órdenes; pata de 1 a 6"], ["0x200 + pata", "Pata a maestra", "Respuestas"]], [2, 2.2, 5])
tabla("Órdenes y respuestas", ["Orden", "Bytes", "Respuesta"], [["PING", "(vacío)", "00 PONG"], ["ARMAR", "01", "01 ACEPTADA o 03 RECHAZADA"], ["IR_A", "02, máscara, tres ángulos int16 en mrad", "ACEPTADA, RECHAZADA o 02 DESARMADA"], ["ESTADO", "03", "05, banderas, tres posiciones en mrad"], ["CERO", "04, máscara", "ACEPTADA o RECHAZADA"], ["DESARMAR", "05", "04 DETENIDA"], ["IDENT", "06", "06 y la MAC de la placa (sin probar en placa)"]], [1.6, 4, 4])
p("La orden IR_A contiene ángulos absolutos respecto del cero del encoder. La pata rechaza valores fuera de rango: coxa de −0,5 a 1,5 rad, fémur de −1 a 1 rad y tibia de −1 a 0,7 rad.")
fig("lucid3.png", "Secuencia de una orden de movimiento por CAN.")
fig("lucid4.png", "Estados de la pata vistos desde el bus. El estado «Sin ID» está implementado y pendiente de probar.", w=14)
h(2, "Seguridad"); li("Sin latido durante 0,5 s, la pata se desarma.", "El PARO detiene todas las articulaciones y tiene la prioridad más alta del protocolo.", "La pata rechaza movimientos con nFAULT, fuera de rango o con el cero del encoder cambiado.", "Al desarmarse, los motores quedan sin par y el fémur cae por gravedad. Es una limitación del diseño actual.")
# ---------------------------------------------------------------- 4
h(1, "Control de la pata")
h(2, "Estructura en cascada"); p("Se usan tres lazos anidados, una arquitectura habitual en servomotores [5]. El lazo externo fija la referencia del interno.")
fig("lucid2.png", "Control en cascada de cada articulación.", w=16)
tabla("Lazos de control", ["Lazo", "Frecuencia", "Controlador", "Medida"], [["Corriente", "1 kHz", "PI; salida en % de PWM a 10 kHz", "IPROPI, bloques de 1 ms"], ["Velocidad", "100 Hz", "PI; salida en A", "Encoder, ventana de 20 ms"], ["Posición", "50 Hz", "P, con límites de velocidad y aceleración", "Encoder"]], [1.6, 1.5, 4, 3])
h(2, "Modelo e identificación")
p("La corriente equivalente se modela con inercia, fricción y gravedad:")
ec(mr("I = "), sub("J", "eq"), mr(" α + "), sub("I", "c"), mr(" sgn(ω) + b ω + G cos(θ + "), sub("θ", "0"), mr(")"))
p("Los parámetros se ajustaron por mínimos cuadrados con rechazo de atípicos (3σ), usando las muestras con |ω| > 0,3 rad/s. Los valores se recalcularon para este informe a partir de los CSV y coinciden con los obtenidos en MATLAB.")
tabla("Parámetros identificados el 19 de noviembre de 2026", ["Articulación", "Jeq (A·s²/rad)", "Ic (A)", "b (A·s/rad)", "G (A)", "θ0 (rad)", "R²"], [[nm, n(ID[k]["J"], 5), n(ID[k]["Ic"], 4), n(ID[k]["b"], 4), n(ID[k]["G"], 4) if ID[k]["G"] else "—", n(ID[k]["th0"], 3) if ID[k]["G"] else "—", n(ID[k]["r2"], 2)] for nm, k in (("Coxa", "coxa"), ("Tibia", "tibia"), ("Fémur, tibia en 0", "femur_t0"), ("Fémur, tibia en −1,55", "femur_tm"), ("Fémur, tibia en +1,55", "femur_tp"))], [2.6, 1.8, 1.2, 1.5, 1.2, 1.2, .9])
fig("fig_ident_femur.png", "Identificación del fémur con la tibia en 0: fricción (izquierda) y gravedad (derecha). Puntos: medidas; línea: modelo.")
p("El coeficiente b negativo de la coxa indica que su fricción baja al aumentar la velocidad (efecto Stribeck). En la sintonía se usó b = 0 en la coxa y la tibia.")
fig("fig_friccion_gravedad.png", "Fricción de Coulomb antes y después de modificar la pata, y gravedad del fémur según la postura de la tibia.")
p(f"La fricción bajó de 0,041 a 0,012 A en el fémur y de 0,059 a 0,013 A en la tibia; en la coxa subió de 0,046 a 0,060 A. La gravedad del fémur cambia entre {n(ID['femur_tp']['G'],3)} y {n(ID['femur_t0']['G'],3)} A según la tibia, lo que justifica un término acoplado.")
h(2, "Diseño de las ganancias"); p("Las ganancias se calcularon por asignación de polos. Para la corriente, con periodo T y tiempo de establecimiento ts:")
ec(sub("K", "i"), mr(" = "), fr(mr("1 − ")+sup(mr("e"), mr("−4T/ts")), mr("a · T")), mr(" ,   "), sub("K", "p"), mr(" = "), sub("K", "i"), mr(" · "), sub("τ", "e"))
p("con a la ganancia de PWM a corriente y τe = 0,2 ms. Para la posición, con control P sobre el lazo de velocidad de retardo equivalente τp:")
ec(sub("K", "p"), mr(" = σ − "), sub("τ", "p"), mr(" "), sup(mr("σ"), mr("2")), mr(" ,   σ = "), fr(mr("4"), mr("ts")))
tabla("Ganancias guardadas en la placa de la pata 1 (21 de noviembre de 2026)", ["Parámetro", "Coxa", "Fémur", "Tibia"], [["Corriente Kp / Ki", "16,032 / 80 161,5", "17,468 / 87 340,2", "14,721 / 73 607,4"], ["Velocidad Kp / Ki", "0,01883 / 0,08187", "0,01989 / 0,11356", "0,01509 / 0,06562"], ["Posición Kp", "3,680", "3,009", "3,680"], ["Límite de corriente (A)", "0,38", "0,30", "0,40"], ["Fricción Ic / Is (A)", "0,0386 / 0,1210", "0,0098 / 0,0700", "0,0083 / 0,0330"], ["GRAV: G (A), θ0 (rad)", "0; 0", "0,0677; −3,100", "0,0102; −2,968"], ["GRAV2: G (A), θ0 (rad)", "—", "0,0070; −2,9487 (tibia)", "0,0070; −2,9487 (fémur)"]], [2.6, 2.4, 2.6, 2.6])
p("Dos valores requieren atención. **Primero**, la ganancia de posición de la coxa guardada (3,680) no es la probada: la relación entre referencia de velocidad y error en el CSV da Kp ≈ 1,94. En el fémur y la tibia, esa misma comprobación da 3,009 y 3,680, iguales a las guardadas. **Segundo**, la gravedad guardada mezcla dos ajustes: GRAV corresponde a uno (0,0677 A, con GRAV2 de 0,0102 A) y GRAV2 a otro (0,0070 A). Además, la tibia tiene un GRAV propio, cuando el modelo calibrado solo usa el término acoplado.")
tabla("Análisis de los lazos con esas ganancias", ["Articulación", "Corriente: cruce / MF", "Velocidad: cruce / MF", "Posición: cruce / MF", "Separación vel./pos."], [[a, f"{n(LZ[a]['corriente'][0],1)} Hz / {n(LZ[a]['corriente'][1],0)}°", f"{n(LZ[a]['velocidad'][0])} Hz / {n(LZ[a]['velocidad'][1],0)}°", f"{n(LZ[a]['posicion'][0])} Hz / {n(LZ[a]['posicion'][1],0)}°", n(LZ[a]['velocidad'][0]/LZ[a]['posicion'][0], 1)+"×"] for a in ("Coxa", "Fémur", "Tibia")], [1.6, 2.3, 2.3, 2.3, 1.9])
p("El cálculo supone retardos de 19 ms en velocidad y 10 ms en posición y no incluye la fricción seca. Los márgenes superan 45°. La separación entre velocidad y posición cumple el criterio de diseño de 4× en el fémur, pero queda en 3,4× en la coxa y la tibia.")
fig("fig_bode_femur.png", "Respuesta en frecuencia calculada de los tres lazos del fémur. Los puntos marcan el cruce por 0 dB.")
h(2, "Compensaciones")
tabla("Compensaciones sumadas a la referencia", ["Compensación", "Comando", "Función"], [["Fricción", "FRICI, Ic, Is", "Ic en movimiento; empujón desde Is cuando está pegada"], ["Forma del empujón", "FRICB", "Sube hasta 2·Is en 200 ms"], ["Mapa de fricción", "FMAP", "Ic por ángulo y sentido; validado solo en simulación"], ["Vibración", "DITHER", "Onda cuadrada en el PWM; 10 % a 80 Hz en las pruebas del 20-nov"], ["Gravedad propia", "GRAV", "G cos(θ + θ0)"], ["Gravedad acoplada", "GRAV2", "Segundo término según otra articulación"]], [2, 2, 5.4])
fig("fig_gravedad_modelo.png", "Modelo de gravedad del fémur ajustado el 20 de noviembre (R² = 0,886; 25 posturas), para tres ángulos de la tibia.")
h(2, "Persistencia"); p("Las ganancias, las compensaciones y la pose de arranque se guardan en NVS. Como el encoder es incremental, no puede recordar el cero: al encender, adopta los ángulos guardados de una pose de reposo. En la prueba de reinicio, las tres configuraciones se recuperaron y el fémur arrancó en +1,0098 rad, el valor guardado. La exactitud depende de que la pata esté en esa misma pose.")
# ---------------------------------------------------------------- 5
h(1, "Resultados experimentales")
h(2, "Comunicación CAN")
tabla("Pruebas de comunicación con una placa (20 de noviembre de 2026)", ["Prueba", "Resultado"], [["Autotest local de cada nodo", "10/10 en la maestra y en la placa"], ["Mensajes pata a maestra", "3/3 con ACK y en orden"], ["1000 pings, firmware de prueba", "1000/1000; 255 / 263 / 1656 µs (mín./prom./máx.)"], ["1000 pings, firmware de control v1, V_MOT encendida y motores quietos", "1000/1000; 4,217 / 9,989 / 10,014 ms"], ["6 pings sueltos, firmware de control v1", "6/6; 8,28 a 8,32 ms"], ["Errores al terminar", "TEC = REC = 0; 0 errores de bus; 0 tramas perdidas"]], [5, 5])
fig("fig_can_latencia.png", "Tiempo de ida y vuelta medido. La línea une mínimo y máximo; el punto es el promedio.")
p("El mínimo medido (255 µs) es coherente con los 204 µs calculados para un ping y su respuesta sin relleno. La latencia del firmware de control v1 no se debía al bus, sino a que la tarea revisaba la cola cada 10 ms. La versión 2 espera bloqueada la llegada de la trama, **pero su latencia todavía no se ha medido**.")
fig("fig_can_carga.png", "Ocupación calculada del bus por el latido, en el peor caso de relleno.")
p(f"Con seis patas, el latido ocuparía como máximo {n(M['can_carga'][5][1],1)} % del bus. Es un cálculo: falta medirlo con las seis placas.")
h(2, "Movimiento ordenado por CAN")
p("El 20 de noviembre, con el protocolo v1 (consignas relativas de ±50 mrad), se aceptaron 265 de 265 consignas, aunque al volver al origen quedaron unos 30 mrad de error por la banda de sostenimiento. El protocolo v2 reemplazó ese esquema por ángulos absolutos. El 21 de noviembre, 7 de 7 órdenes llegaron, en 0,7 a 1,5 s, con un error de hasta 19 mrad (tolerancia de 20 mrad, 2 rad/s y 5 rad/s²).")
fig("fig_can_ordenes.png", "Error en el momento de llegada de cada movimiento ordenado por CAN. Morado: fémur; verde: tibia; franja: tolerancia.")
p("Las siete órdenes equivalen a nueve movimientos de articulación, porque dos movieron fémur y tibia a la vez. La coxa no participó por la falla de su driver.")
h(2, "Seguimiento de posición")
tabla("Desempeño en posición de la pata actual (7 movimientos por articulación)", ["Articulación", "Llegaron", "Tiempo medio (s)", "Rango (s)", "Error final máx. (mrad)", "Sobrepaso máx. (%)"], [[nm, f"{d['llegaron']}/{d['n']}", n(d["t_media"]), f"{n(d['t_min'])}–{n(d['t_max'])}", n(d["efin_max"]*1000, 1), n(d["sob_max"], 1)] for nm, d in (("Coxa (19-nov)", c), ("Fémur (20-nov)", fc), ("Tibia (20-nov)", tc))], [2, 1.2, 1.7, 1.8, 2, 1.8])
p("Condiciones: lazo a 2 rad/s y 15 rad/s² de límite y tolerancia de 10 mrad; una sola repetición. El control vuelve a corregir cuando el error supera 2 × tolerancia, por lo que el error final puede quedar entre 10 y 20 mrad. En la coxa, el primer movimiento terminó con 28,8 mrad.")
fig("fig_resp_femur.png", "Fémur: posición, velocidad y PWM (20 de noviembre, gravedad calibrada y dither de 10 %).", w=12.6)
fig("fig_resp_tibia.png", "Tibia: posición, velocidad y PWM (20 de noviembre, dither de 10 %).", w=12.6)
fig("fig_resp_coxa.png", "Coxa: posición, velocidad y PWM (19 de noviembre, Kp de posición ≈ 1,94).", w=12.6)
fig("fig_desempeno.png", "Tiempo de llegada (la línea es la media) y error final de cada movimiento.")
h(2, "Efecto del empujón de arranque")
p(f"Con Is = 0,036 A, el fémur completó {fa['llegaron']} de {fa['n']} movimientos: al bajar, el empujón no superaba la gravedad compensada y la articulación no se soltaba. Con Is = 0,070 A completó {fb['llegaron']} de {fb['n']}, con un tiempo medio de {n(fb['t_media'])} s.")
fig("fig_femur_empujon.png", "Fémur antes y después de aumentar la corriente de arranque Is (19 de noviembre).")
p(f"Con la gravedad recalibrada y dither, el error final máximo bajó de {n(fb['efin_max']*1000,1)} a {n(fc['efin_max']*1000,1)} mrad. Como se cambiaron las dos cosas a la vez, y también el rango de movimiento (1,8 y 1,1 rad), no se puede atribuir la mejora a una sola.")
h(2, "Evolución del control")
tabla("Evolución entre versiones de la pata", ["Fecha", "Cambio", "Resultado"], [["12-nov", "Pata anterior; fricción en la referencia de corriente", "Fémur 6/6, tibia 6/6, coxa 5/6; sobrepaso hasta 14 %"], ["14-nov", "Empujón FRICB, dither y corrección de FRIC_LOOSE_RAD", "Fémur 8/8, coxa 7/8; sobrepaso ≤ 1 % salvo un caso de 16 % en la tibia"], ["19-nov", "Pata modificada; nueva identificación", "Coxa 7/7; fémur de 4/7 a 7/7 al subir Is; tibia 6/8 (dos objetivos en topes)"], ["20-nov", "Gravedad calibrada y acople de la tibia apretado", "Fémur 7/7 y tibia 7/7"], ["21-nov", "CAN v2 y persistencia en NVS", "7/7 órdenes por CAN"]], [1.2, 4, 4.6])
fig("fig_evolucion.png", "Comparación del 12 de noviembre con las pruebas del 19 y 20 de noviembre.")
p("La comparación es orientativa: cambiaron la mecánica, los rangos y los límites de aceleración. El sobrepaso bajó en la coxa y la tibia y los tiempos medios no mejoraron en la coxa ni en el fémur.")
# ---------------------------------------------------------------- 6
h(1, "Fallas encontradas y diagnóstico")
tabla("Fallas, diagnóstico y estado", ["Síntoma", "Diagnóstico", "Estado"], [["El bus queda en dominante (CRX = 0) con una MOD_DRV", "El módulo entregaba 3,28 V en CRX: unión abierta entre J4 pin 4 e IO22", "Placa apartada; falta un puente de alambre"], ["nFAULT en la coxa de otra placa", "Aparece solo con V_MOT: la falla la reporta el driver DRVA o su entorno", "Causa sin confirmar; falta medir VM y VCP"], ["Reinicios por watchdog en la maestra", "pdMS_TO_TICKS(5) daba 0 con FreeRTOS a 100 Hz", "Corregido: espera mínima de 1 tick"], ["Respuesta CAN de 8 a 10 ms", "La tarea revisaba la cola y dormía un tick de 10 ms", "Cambiado en v2; falta medirlo"], ["El fémur no baja (4/7)", "Empujón insuficiente frente a la gravedad compensada", "Resuelto con Is = 0,070 A"], ["La tibia llega a −2,44 rad", "Error de tecleo en el rango (−14) y acople flojo", "Acople apretado; el dashboard limita el rango"], ["Articulaciones a saltos", "Stick-slip por los bujes de PLA de la pata anterior", "Menor en la pata modificada; rodamientos pendientes"]], [3, 4, 3])
# ---------------------------------------------------------------- 7
h(1, "Limitaciones")
li("Las pruebas son de una pata, en banco, y con una repetición por condición: no hay análisis estadístico.", "La escala del encoder (2800 cuentas por vuelta) no está verificada; si es incorrecta, todos los ángulos cambian en la misma proporción.", "Las corrientes no son la corriente real del motor, sino su equivalente por ciclo de trabajo.", "El Kp de posición guardado en la coxa no se ha probado en la pata, y su driver está en falla en la placa usada con CAN.", "La gravedad guardada mezcla dos calibraciones.", "El CAN no se ha probado con motores en movimiento continuo, cable largo ni más de una pata, y no se midió la resistencia entre CANH y CANL.", "El ID configurable y la maestra para seis patas compilan, pero no se han probado en placas. La interfaz web de una pata se usó con la pata real; su versión para seis patas se probó solo con una maestra simulada.", "El desarme por pérdida del latido está implementado, pero no se probó cortando la comunicación.", "Los tests en C del firmware no se ejecutaron tras los últimos cambios, porque el equipo no tiene compilador de PC.", "El modelo de los lazos es lineal y no incluye fricción seca ni saturación.")
h(1, "Conclusiones")
li(f"El control en cascada con compensaciones llevó el fémur y la tibia a 7 de 7 objetivos, con error final de hasta {n(tc['efin_max']*1000,1)} mrad y sobrepaso de hasta {n(max(fc['sob_max'],tc['sob_max']),1)} %.", "La coxa es la articulación más lenta y con mayor error; su fricción de Coulomb es entre tres y cinco veces la de las otras dos.", "La gravedad del fémur depende de la tibia, por lo que se necesita un término acoplado.", "El bus CAN respondió 1000 de 1000 mensajes sin errores con una pata, y la carga calculada para seis es baja.", "La maestra puede ordenar movimientos con un solo mensaje, y la pata los ejecuta con su propio control.")
h(1, "Trabajo pendiente")
li("Reparar el driver de la coxa y probar sus ganancias guardadas.", "Enviar y guardar una sola calibración de gravedad.", "Verificar la escala del encoder girando 90°.", "Medir la latencia de CAN v2 y repetir las pruebas con motores en movimiento.", "Asignar IDs y probar las seis placas, primero por separado y luego en el bus.", "Repetir cada movimiento al menos diez veces para reportar media y desviación.", "Hacer un homing contra un tope e integrar la cinemática inversa en la maestra.")
hs("Referencias")
for i, t in enumerate(["Texas Instruments, “DRV8876 H-Bridge Motor Driver With Integrated Current Sense and Regulation,” hoja de datos SLVSDS7.", "ISO 11898-1, “Road vehicles — Controller area network (CAN) — Part 1: Data link layer and physical signalling.”", "Texas Instruments, “SN65HVD23x 3.3-V CAN Bus Transceivers,” hoja de datos SLOS346.", "R. I. Davis, A. Burns, R. J. Bril y J. J. Lukkien, “Controller Area Network (CAN) schedulability analysis: Refuted, revisited and revised,” Real-Time Systems, vol. 35, n.º 3, pp. 239–272, 2007.", "Z. Wang, Z. Chen y J. Zhang, “Servo Motor Position Control Based on DSP,” en Mechanical Engineering and Technology, Springer, 2012."], 1): p(f"[{i}] {t}", "Referencia")
h(1, "Anexo: trazabilidad de los datos")
tabla("Origen de los resultados (repositorio FFrank456/Tesis_hexapodo)", ["Resultado", "Fuente"], [["Respuesta de la coxa", c["archivo"]], ["Fémur, tres pruebas", f"{fa['archivo']}; {fb['archivo']}; {fc['archivo']}"], ["Tibia", tc["archivo"]], ["Identificación", "rampas y vaivenes del 19-nov en matlab/datos"], ["Pruebas CAN", "02-electronica/notas/prueba-modulos-can-esp32-wcmcu230.md (valores anotados; no hay registro crudo)"], ["Calibración de gravedad y evolución", "docs/identificacion_pata_modificada_2026-11-19.md y docs/reporte_control_cascada.md"], ["Protocolo", "pata_MOD_DRV/main/can_protocol.h, commit 6644ac9"], ["Figuras y métricas", "05-documentacion/informe-unidad2/generar_figuras.py y metricas.json"]], [3, 7])

NS = 'xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math" xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture"'
XH = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
doc = f'{XH}<w:document {NS}><w:body>{"".join(B)}<w:sectPr><w:footerReference w:type="default" r:id="rFoot"/><w:pgSz w:w="11906" w:h="16838"/><w:pgMar w:top="1418" w:right="1418" w:bottom="1418" w:left="1418" w:header="709" w:footer="709" w:gutter="0"/><w:titlePg/></w:sectPr></w:body></w:document>'
def est(i, nom, ppr="", rpr="", base="Normal", extra=""): return f'<w:style w:type="paragraph" w:styleId="{i}"><w:name w:val="{nom}"/>{f"<w:basedOn w:val={chr(34)}{base}{chr(34)}/>" if base else ""}{extra}<w:qFormat/><w:pPr>{ppr}</w:pPr><w:rPr>{rpr}</w:rPr></w:style>'
F = '<w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman" w:cs="Times New Roman"/>'
nx = '<w:next w:val="Normal"/>'
sty = f'{XH}<w:styles {NS}><w:docDefaults><w:rPrDefault><w:rPr>{F}<w:sz w:val="23"/><w:lang w:val="es-PE"/></w:rPr></w:rPrDefault></w:docDefaults><w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/><w:pPr><w:spacing w:after="140" w:line="320" w:lineRule="auto"/><w:jc w:val="both"/></w:pPr></w:style>' + \
 est("Heading1", "heading 1", '<w:keepNext/><w:numPr><w:ilvl w:val="0"/><w:numId w:val="1"/></w:numPr><w:spacing w:before="400" w:after="160"/><w:jc w:val="left"/><w:outlineLvl w:val="0"/>', F+'<w:b/><w:color w:val="1F3A5F"/><w:sz w:val="32"/>', extra=nx) + \
 est("Heading2", "heading 2", '<w:keepNext/><w:numPr><w:ilvl w:val="1"/><w:numId w:val="1"/></w:numPr><w:spacing w:before="280" w:after="120"/><w:jc w:val="left"/><w:outlineLvl w:val="1"/>', F+'<w:b/><w:color w:val="1F3A5F"/><w:sz w:val="26"/>', extra=nx) + \
 est("SinNumero", "Titulo sin numero", '<w:keepNext/><w:spacing w:before="300" w:after="160"/><w:jc w:val="left"/>', F+'<w:b/><w:color w:val="1F3A5F"/><w:sz w:val="30"/>', extra=nx) + \
 est("Title", "Title", '<w:spacing w:before="600" w:after="300"/><w:jc w:val="center"/>', F+'<w:b/><w:color w:val="1F3A5F"/><w:sz w:val="44"/>') + est("Portada", "Portada", '<w:spacing w:after="160"/><w:jc w:val="center"/>', '<w:sz w:val="26"/>') + \
 est("Caption", "caption", '<w:spacing w:before="80" w:after="200" w:line="240" w:lineRule="auto"/><w:jc w:val="center"/>', '<w:sz w:val="20"/>') + est("Imagen", "Imagen", '<w:keepNext/><w:spacing w:before="160" w:after="60" w:line="240" w:lineRule="auto"/><w:jc w:val="center"/>') + \
 est("Celda", "Celda", '<w:spacing w:before="40" w:after="40" w:line="240" w:lineRule="auto"/><w:jc w:val="left"/>', '<w:sz w:val="19"/>') + est("Vineta", "Vineta", '<w:numPr><w:ilvl w:val="0"/><w:numId w:val="2"/></w:numPr><w:spacing w:after="80"/>') + \
 est("Ecuacion", "Ecuacion", '<w:tabs><w:tab w:val="center" w:pos="4535"/><w:tab w:val="right" w:pos="9070"/></w:tabs><w:spacing w:before="100" w:after="160"/><w:jc w:val="left"/>') + est("Referencia", "Referencia", '<w:ind w:left="480" w:hanging="480"/><w:spacing w:after="100" w:line="260" w:lineRule="auto"/><w:jc w:val="left"/>') + \
 "".join(est(f"TOC{k}", f"toc {k}", f'<w:tabs><w:tab w:val="right" w:leader="dot" w:pos="9070"/></w:tabs><w:spacing w:after="40" w:line="250" w:lineRule="auto"/><w:ind w:left="{(k-1)*260}"/><w:jc w:val="left"/>') for k in (1, 2)) + est("TableofFigures", "table of figures", '<w:tabs><w:tab w:val="right" w:leader="dot" w:pos="9070"/></w:tabs><w:spacing w:after="40" w:line="250" w:lineRule="auto"/><w:ind w:right="500"/><w:jc w:val="left"/>', '<w:sz w:val="21"/>') + "</w:styles>"
num = f'{XH}<w:numbering {NS}><w:abstractNum w:abstractNumId="0"><w:multiLevelType w:val="multilevel"/><w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="decimal"/><w:pStyle w:val="Heading1"/><w:lvlText w:val="%1."/><w:pPr><w:ind w:left="480" w:hanging="480"/></w:pPr></w:lvl><w:lvl w:ilvl="1"><w:start w:val="1"/><w:numFmt w:val="decimal"/><w:pStyle w:val="Heading2"/><w:lvlText w:val="%1.%2."/><w:pPr><w:ind w:left="620" w:hanging="620"/></w:pPr></w:lvl></w:abstractNum><w:abstractNum w:abstractNumId="1"><w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="bullet"/><w:lvlText w:val="•"/><w:pPr><w:ind w:left="500" w:hanging="280"/></w:pPr></w:lvl></w:abstractNum><w:num w:numId="1"><w:abstractNumId w:val="0"/></w:num><w:num w:numId="2"><w:abstractNumId w:val="1"/></w:num></w:numbering>'
foot = f'{XH}<w:ftr {NS}><w:p><w:pPr><w:jc w:val="center"/></w:pPr>{fld("PAGE", "1")}</w:p></w:ftr>'
R = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
rel = f'{XH}<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rSty" Type="{R}/styles" Target="styles.xml"/><Relationship Id="rNum" Type="{R}/numbering" Target="numbering.xml"/><Relationship Id="rFoot" Type="{R}/footer" Target="footer1.xml"/>' + "".join(f'<Relationship Id="rImg{i}" Type="{R}/image" Target="media/img{i}.png"/>' for i in range(1, len(IMG)+1)) + "</Relationships>"
ct = f'{XH}<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Default Extension="png" ContentType="image/png"/>' + "".join(f'<Override PartName="/word/{a}.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.{b}+xml"/>' for a, b in (("document", "document.main"), ("styles", "styles"), ("numbering", "numbering"), ("footer1", "footer"))) + "</Types>"
sal = os.path.join(A, "Informe_Unidad2_Control_CAN.docx")
with zipfile.ZipFile(sal, "w", zipfile.ZIP_DEFLATED) as z:
    z.writestr("[Content_Types].xml", ct); z.writestr("_rels/.rels", f'{XH}<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="r1" Type="{R}/officeDocument" Target="word/document.xml"/></Relationships>')
    z.writestr("word/document.xml", doc); z.writestr("word/styles.xml", sty); z.writestr("word/numbering.xml", num); z.writestr("word/footer1.xml", foot); z.writestr("word/_rels/document.xml.rels", rel)
    for i, r in enumerate(IMG, 1): z.write(r, f"word/media/img{i}.png")
print(sal, CNT)
