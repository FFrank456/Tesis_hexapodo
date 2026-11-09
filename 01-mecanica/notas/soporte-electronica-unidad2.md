---
tipo: componente-tecnico
area: mecanica
unidad: Unidad 2
fecha: 2026-11-04
version: v2
archivo_cad: ROBOT_ESAMBLE › Electronica_U2 (Fusion, proyecto "Tesis hexápodo")
estado: diseñado
tags:
  - mecanica
  - impresion-3d
  - unidad2
  - electronica
---

# Soporte de la electrónica de la Unidad 2 (v2)

Modelado en Fusion dentro de ROBOT_ESAMBLE, componente `Electronica_U2`. Los STL exportados (`Marco_nivel_PETG`, `Parante_base_05`, `Parante_ext_04/08`, `Tapa_PETG`) están en `01-mecanica/cad/soporte-electronica-u2/`.

## Decisiones del usuario
- 6 MOD_DRV (una por pata) y la placa maestra ESP32-S3-DevKitC-1. La potencia ([[Modulo de control de extremidad - Registro del proyecto|PCBicho 6× MP1584]]) va dentro del chasis y las placas encima.
- La PCB MOD_DRV **no se puede modificar**, así que se monta en un marco impreso.
- Entrada de 12 V por **J1** (bornera de 2 polos, paso de 3,5 mm). J3A, J3B y J3C llevan motor y encoder juntos.
- Alimentación y **bus CAN** con un mazo en cadena. PETG con insertos M3 termofijados. El acceso para reprogramar es ocasional.
- **v2:** la distribución sigue la forma del chasis, con **3 niveles × 2 placas** (una adelante y otra atrás).

## Restricción geométrica
- Placa de 63,5 × 51,5 mm. Por arriba sobresalen 8,6 mm (J1, P3); por abajo, C27 sobresale 10,6 mm.
- Separación mínima entre placas: 15,6 mm (C27 contra el RESET de la placa de abajo).
- Techo del chasis: octágono alargado de ~100 × 70 mm (x −14…86, y 46…116, borde superior en z = 3,45).

## Diseño v2 (mm, coordenadas del ensamble, Z hacia arriba, frente = +X)
- Placas giradas 90°, con el lado largo (el de J3) en Y. La delantera tiene J3 hacia +X (x 39…90,5) y la trasera hacia −X (x −18,5…33). Queda un hueco central de 6 mm con las J1 enfrentadas, por donde pasa el mazo.
- Asignación: N1 = L1 adelante / L3 atrás; N2 = L6 / L4; N3 = L2 / L5.
- **Marco_nivel** (×3): octágono de 114,8 × 79,5 mm, esquinas a 45° de 13 mm, 2,5 mm de espesor, recortes para los conectores y 8 agujeros M3.
- Separadores en coordenadas locales de cada placa: (−4,5; 12), (−4,5; 51,5), (68; 12), (68; 46).
- **Bandeja_base:** el mismo octágono, de 3 mm, apoyado en el borde del chasis (z 3,45…6,45). Lleva 8 agujeros de Ø4 para insertos y una ranura de 10 × 42 mm.
- **Separadores_M3:** 8 de 10 mm y 16 de 15 mm. Separación entre placas: 17,5 mm. Holguras de ~1,9 mm (C27/RESET) y ~2 mm (C27/bandeja).
- Altura: la cara superior del nivel 3 queda en z ≈ 54,4 y los componentes llegan a z ≈ 62,9.

## Pendiente
- [ ] Posición de la placa maestra (el soporte en L anterior quedó oculto porque choca con la v2).
- [ ] Retención vertical de cada placa en su marco.
- [ ] Fijación de la bandeja al borde del chasis.
- [ ] Los cuerpos son *base features*, no paramétricos.

> [!question] Pregunta abierta
> ¿Dónde va la maestra ESP32-S3: encima del nivel 3 o en la tapa? Eso define la longitud del bus CAN y dónde quedan los dos extremos con 120 Ω.

Relacionado: [[arquitectura-electronica-hexapodos]] · [[urdf-pose-cero-hexapodo]]
