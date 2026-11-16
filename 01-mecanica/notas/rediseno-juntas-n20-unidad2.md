---
tipo: componente-tecnico
area: mecanica
unidad: Unidad 2
fecha: 2026-11-14
version: v3
archivo_cad: ROBOT_ESAMBLE_alfred Copy (Fusion, Tesis hexápodo › Preparaciones_to_ROS2)
estado: en-diseño
tags:
  - mecanica
  - unidad2
  - n20
  - juntas
  - chasis
---

# Rediseño de las juntas N20 y del chasis – pata 1 (Unidad 2)

Archivo Fusion **ROBOT_ESAMBLE_alfred Copy**. Contiene la pata 1 (coxa_link_1, femur_link_1, tibia_link_1), el componente `base_J0`, el `Chasis` y la electrónica de referencia. Primero se rediseña una pata; las otras cinco se copiarán cuando esta quede validada.

## Concepto: acople directo con apoyo en la carcasa del motor
- El micromotor Pololu HPCB 12 V (formato N20) es coaxial con la junta. No se usan engranajes impresos, solo la caja reductora del N20; se evalúa pasar de 100:1 a 380:1.
- **El motor va siempre en el eslabón padre**, dentro de un cubo de alojamiento (en J1 `sup_articulacion`, en J2 el cubo integrado en `femur_link` y en J0 `soporte_J0_chasis`).
- **Apoyo 1, carcasa:** el eslabón hijo gira sobre el cubo mediante el anillo `ajuste_2`, en PETG con buje de bronce.
- **Apoyo 2, eje:** el eslabón hijo se fija al eje D de 3 mm con una brida tipo Pololu: agujero de Ø3, cubo de Ø10 × 10 mm, disco de Ø22 × 2 mm y 4 agujeros M3. El disco se aloja en un rebaje del fondo de la copa con 0,2 mm de holgura radial.
- **Apoyo 3, parte libre del cubo de la brida:** una placa de 3 mm con buje de Ø10,4 sostiene el extremo del eje del lado opuesto.
  - En J0 es `soporte_J0_inferior`, empotrado en el fondo del pilar del chasis.
  - En J1 es `soporte_J1_buje` y en J2 `soporte_J2_buje`. Cada uno se fija con un solo M3 × 8 y una tuerca cuadrada, en una ranura pasante de 5,8 × 2,2 mm de la placa.

## Chasis (referencia: la tapa PETG de la electrónica)
- **Forma:** octógono simétrico de 100 × 71,8 mm por fuera (x −14…86, y 45,08…116,92). El contorno exterior sigue la cara inferior de `Tapa_PETG`, que se apoya encima. Pared de 3 mm, piso de 2,1 mm y altura de −13,55 a 13,3 mm.
- **Esquinas:** a 45°, iguales en los cuatro lados, con 16 mm por lado y una diagonal interior de unos 20 mm.
- **Pilares:** son 6, cuatro en las esquinas y dos al centro de los lados (en x = 31).
  - Cada pilar mide 18 mm de ancho, con paredes de 1,2 mm, fondo plano de 2 mm y pared trasera. No llevan rampa.
  - El M3 atraviesa de arriba abajo, y hay 4 ranuras pasantes de 5,8 × 2,5 mm cada 7,95 mm.
  - El fondo tiene 2 agujeros Ø3,4 para los tornillos que entran desde abajo.
- **Agujeros de la tapa:** los pilares de las esquinas del oeste coinciden con los agujeros Ø3,4 de `Tapa_PETG`. Los del este están en x = 81,27, así que hay que mover 4,9 mm los 2 agujeros del este de la tapa.
- **Aligeramiento:** 6 ventanas en el piso, de unos 20 × 22 mm con esquinas de radio 4, que reducen el volumen un 12 %.

## Coxa y J0 (cadera)
- **Componente `base_J0`:** está fijo y contiene el motor de J0, `soporte_J0_chasis` y `soporte_J0_inferior`.
  - `soporte_J0_chasis`: cubo de alojamiento del motor, brazo superior y columna de 15,2 mm que entra recta en el pilar con 0,2 mm de holgura. El M3 pasante es M3 × 40 con tuerca por debajo.
  - `soporte_J0_inferior`: buje alrededor del cubo de la brida y brazo empotrado en el fondo del pilar, fijado con 2 M3 desde abajo que roscan en la columna.
- **`brazo_J0`:** gira con la pata y lleva el anillo de apoyo sobre el cubo, el asiento de la brida (4 M3 desde abajo) y la placa del brazo.
- **Alineación del eje de J0:** vertical y alineado con el agujero M3 del pilar, a 23,03 mm de él sobre la línea de centro del pilar.
- **Unión `sup_articulacion` ↔ `brazo_J0`:** U con macho de 9 × 9 mm, sin agujeros sobrantes. La ranura tiene 0,15 mm de holgura adicional por la inclinación que tenía el brazo.

## Fémur (`femur_link`)
- Es una sola pieza: la copa de J1 con el anillo integrado, la placa y el cubo de J2.
- Lleva 2 ventanas pasantes de aligeramiento, que reducen el volumen un 20 %.
- El motor de J2 está alineado con el de J1.

## Tibia
- **Unión con el macho `inf_articulacion`:** es una U cerrada. La parte amarilla `tibia_link_base_A` rodea el macho por los cuatro lados con tapas de 2 mm y 0,2 mm de holgura.
  - Las 4 ranuras de tuerca del macho son pasantes, así que las tuercas se colocan antes de ensamblar.
  - El cuello del macho tiene una ventana de aligeramiento, que reduce el volumen un 15 %.
- **Forma tipo tarántula en tramos rectos (v3):** el giro total es de 60°, repartido en dos rodillas de 30° redondeadas con R5. La sección es de 15 × 10 mm.
  - **Tramo 1:** recto hasta 63 mm del eje de J2, con un agujero de aligeramiento de Ø10.
  - **Tramo 2:** 30 mm a −30°, con una ranura de 10 × 18,5 mm.
  - **Tramo 3:** 15 mm a −60°.
  - **Comprobación:** al girar J2 de −120° a +120° no choca con el fémur.
- **Pie (`soporte_tibia_A`):** casco ovalado de 27 × 23 × 19,5 mm que se calza sobre la punta.
  - Se fija con un M3 × 22 transversal, con alojamiento para la cabeza y hexágono para la tuerca.
  - El centro de la punta queda a 96 mm hacia adelante y 27 mm hacia abajo de J2.
  - Se recomienda imprimirlo en TPU, o en PETG con 0,2 mm de holgura.
- **Pernos de la U:** 2 M3 con la cabeza en la tapa superior (alojamiento de Ø5,8 × 1,8 mm) y la tuerca hexagonal en la tapa inferior (5,7 mm entre caras, 1,8 mm de profundidad).

## Juntas (Fusion)
| Junta | Tipo | Eje |
|---|---|---|
| `join_coxa_1_J0` | Revolución, según lo construido (coxa ↔ `base_J0`) | Eje del motor de J0 |
| `join_femur_1_J1` | Revolución, según lo construido (fémur ↔ coxa) | Eje del motor de J1 |
| `join_tibia_1_J2` | Revolución, según lo construido (tibia ↔ fémur) | Eje del motor de J2 |

Las juntas antiguas (`join_femur_1` y `join_tibia_1`) giraban sobre un eje desplazado unos 2,5 mm y se eliminaron. Con las nuevas, la brida no se separa del eje en ningún ángulo; se comprobó en la pose J1 −81° / J2 63°.

## Análisis de fuerzas y colisiones
- **Cargas:** robot de unos 1,2 kg apoyado en 3 patas, es decir 3,9 N por pata en reposo y 7,8 N con un factor dinámico de 2. El par máximo de diseño es el de bloqueo del N20 380:1, 0,49 N·m.
- **Placas de fémur y tibia** (vigas de 29 × 9 mm):

| Sección | Módulo resistente | Esfuerzo máx. |
|---|---|---|
| Maciza | 391 mm³ | 1,3 MPa |
| Con ventanas | 108 mm³ | 4,5 MPa |

  El factor de seguridad es mayor que 5 frente a 25–40 MPa del PETG impreso.
- **Barrido de colisiones:** J1 y J2 giran libres en ±90° (J2 hasta ±120° con la tibia en arco). J0 gira libre en ±90°; a ±105° la coxa roza el soporte del motor, así que se recomienda limitar J0 a ±90°.
- **Superposiciones fijas corregidas:** el anillo y la copa del fémur ahora son un solo cuerpo, y las bridas de J1 y J2 tienen su rebaje en la copa.

## Cambios del 14-nov (tarde)
- **Diámetro de las juntas:** todas las piezas redondas que rodean un acople bajaron a Ø22, el diámetro de la brida.
  - Son las copas y aros de J1 y J2, `soporte_J1_buje`, `soporte_J2_buje` y los aros de `brazo_J0`.
  - En `brazo_J0` se mantiene la unión aro–brazo con 9 mm de ancho, y se rellenó la canaleta de la cara interior del brazo.
- **Agujeros de las bridas:** en `brazo_J0` (J0) y en `inf_articulacion` (J2) se alinearon con la brida. Antes estaban desfasados 20° y 24,3°.
- **Unión `brazo_J0` ↔ `sup_articulacion`:** los 2 pernos M3 quedan a 21,2 y 30,3 mm del eje de J0. La cabeza va arriba (Ø5,8 × 3 mm) y la tuerca hexagonal abajo (2,4 mm).
- **Fémur dividido:** `femur_tapa_J1` (copa de J1 + franja de 7 mm) se une a `femur_link` con un M3 × 12. Tiene la cabeza en la cara exterior (Ø5,8 × 3) y la tuerca en una ranura pasante en la mitad de la placa.
- **Alojamientos de cabeza Ø5,8 × 3 mm:** en `femur_tapa_J1` y en `inf_articulacion` (en el perno hacia el macho).
- **Ranuras de tuerca:**
  - `inf_articulacion (1)`: ranura de 5,8 × 2,6 mm para el perno de J2.
  - `inf_articulacion`: agujeros del acople J2 de Ø3,2 con alojamiento de Ø6,2 × 3 mm.
- **`soporte_J0_chasis`:** separado otra vez de la carcasa del motor J0 (`J0_Cuerpo4`).
- **Aligeramiento del fémur:** 4 agujeros de Ø8 en la placa central de `femur_link`, que bajan su volumen de 10 701 a 8 891 mm³.
  - El resto de piezas no tiene zonas macizas con al menos 2,5 mm de pared hasta los agujeros, ranuras o alojamientos cercanos.

## Pendientes
- Revisar `Empalme13` y `Empalme14` de la tibia, que se hicieron sobre la barra anterior.
- Recrear `join_femur_1_J1` y `join_tibia_1_J2`; en el modelo solo queda `join_coxa_1_J0`.
- Mover los 2 agujeros del este de `Tapa_PETG` a x = 81,27.
- Aplicar límites de ±90° a las tres juntas.
- Errores antiguos en el historial: `Ajuste_soporte_chasis`, `Ajuste_soporte_chasis_rect`, `Quitar_brazo_interior`, `Boceto39–41` y `Dividir1`.
- Confirmar el círculo de agujeros de la brida real (se supuso Ø16,3).
- Exportar los STL de la pata 1 y replicar el diseño en las otras cinco patas.

Ver también [[analisis-esfuerzos-diseno-hexapodo]].
