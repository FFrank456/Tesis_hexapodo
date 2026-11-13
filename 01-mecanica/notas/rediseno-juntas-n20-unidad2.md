---
tipo: componente-tecnico
area: mecanica
unidad: Unidad 2
fecha: 2026-11-13
version: v1
archivo_cad: ROBOT_ESAMBLE_alfred Copy (Fusion, Tesis hexápodo › Preparaciones_to_ROS2)
estado: en-diseño
tags:
  - mecanica
  - unidad2
  - n20
  - juntas
---

# Rediseño de las juntas N20 – pata 1 (Unidad 2)

Archivo Fusion **ROBOT_ESAMBLE_alfred Copy**. Contiene solo la pata 1 (coxa_link_1, femur_link_1, tibia_link_1) y la electrónica de referencia. Primero se rediseña una pata; las otras cinco se copiarán cuando esta quede validada.

## Concepto: acople directo con apoyo en la carcasa del motor
- El micromotor Pololu HPCB 12 V (formato N20) es coaxial con la junta. No hay engranajes impresos: solo se usa la caja reductora del N20, y se evalúa pasar de 100:1 a 380:1.
- **Apoyo 1, carcasa del motor:** el link padre tiene un cubo (`sup_articulacion`) que aloja el motor y lleva una ranura. En esa ranura entra el anillo `ajuste_2` del link hijo, que va sobre un buje de bronce y se imprime en PETG.
- **Apoyo 2, eje del motor:** la copa del link hijo se fija al eje D de 3 mm con una brida tipo Pololu: agujero de Ø3, cuerpo de Ø10 × 10 mm, disco de Ø22 y 4 agujeros M3.
- Los motores de las tres juntas quedan alineados en el mismo eje axial. Las juntas están desplazadas 10 mm (`join_femur_1` y `join_tibia_1`).

## Coxa (`coxa_link_1`)
- **J0 (cadera):** se rediseñó con el mismo concepto que J1. El brazo `brazo_J0` es la placa del cubo de J1 girada 90°. Tiene el alojamiento del N20 vertical con el eje hacia arriba (hacia la base), el cubo con ranura y una escotadura de R 12,8 mm para la copa de la base.
- **Referencias de J0:** quedan modelados el motor (cuerpos `J0_*`), el anillo `ajuste_2_J0_base` y `Brida_acople_3mm_J0`. El anillo y la brida le corresponden al `base_link`, que todavía hay que adaptar.
- **Posición de J0:** el eje se desplazó 2 mm para quedar centrado en la placa de J1.
- **Unión del cubo de J1 con el brazo de J0:** es una U con macho y pernos.
  - El macho (9 × 9 × 15 mm) sale de `sup_articulacion`, al ras de las dos caras. Lleva agujeros internos de Ø3,4 y 4 ranuras para tuercas cuadradas M3 (5,8 × 2,2 mm), que se abren por la cara lateral.
  - La U está en `brazo_J0`, con 0,2 mm de holgura por lado y 2 agujeros pasantes de Ø3,4.
- Se cerró el agujero lateral de Ø3,5 que no tenía función. El brazo antiguo quedó oculto como `sup_articulacion (1)_orig`.

## Fémur (`femur_link_1`)
- Es una sola pieza, `femur_link`: la copa de J1 y el cubo de J2 unidos por una placa maciza. Se quitó la viga intermedia `femur_tibia`.
- Se rellenaron las ventanas rectangulares, el agujero irregular, los agujeros en D, la V y los agujeros laterales. Las caras quedaron lisas, sin escalones ni marcas (`Rellenar_placa_femur`).
- El motor de J2 se alineó con el de J1 (`Alinear_motor_femur_J2`).

## Tibia (`tibia_link_1`)
- **J2:** es una copia de J1, con la copa `inf_articulacion`, `ajuste_2` y `Brida_acople_3mm_J2`.
- **Unión de la copa con la base:** es una U con macho y pernos.
  - El macho es simétrico (13 × 9 mm) y lleva agujeros internos y 4 ranuras para tuercas cuadradas.
  - La U de `tibia_link_base_A` tiene brazos de 3 mm y 4 agujeros pasantes de Ø3,4.
- La copa y el cuello se rellenaron y quedaron simétricos (`Rellenar_centrar_tibia`).
- El macho y la base de la tibia se centraron en la placa, entre `ajuste_2` y el acople (`Centrar_base_tibia_placa`, −2,4 mm).

## Pendientes
- Verificar `brazo_J0` contra el chasis y rediseñar el `base_link` para J0: la copa alrededor del cubo, el anillo `ajuste_2_J0_base` y la brida.
- Corregir los choques que quedaron en las copas: en J1, `femur_link` choca con `ajuste_2 (1)` (134 mm³) y con `Brida_acople_3mm_J1` (124 mm³); en J2 hay los mismos choques. Hay que reabrir el alojamiento de la brida (Ø10,2) y la holgura del anillo.
- `Dividir1` (coxa) tiene una referencia perdida. `Boceto39`, `Boceto40` y `Boceto41` perdieron su plano. `CarasDesfase4` y `CarasDesfase8` están vacíos.
- Confirmar el círculo de agujeros de la brida real (se supuso Ø16,3).
- Exportar los STL de la pata 1 y replicar el diseño en las otras cinco patas.

Ver también [[analisis-esfuerzos-diseno-hexapodo]]. Análisis de fuerzas y torque por link: artifact "fuerzas-pata-u2".
