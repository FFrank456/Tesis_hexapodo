# ensayo_torque_N20

Banco interactivo para medir el torque de un N20 con la MOD_DRV. Sigue la [guía del ensayo](../../05-documentacion/guia-ensayo-torque/guia_ensayo_torque_N20.pdf), pero la placa da los pulsos, lee el encoder y el panel grafica, calcula y guarda.

| Carpeta | Contenido |
|---|---|
| `main/` | Firmware ESP-IDF 5.4.4 (encoder y sensado copiados de `pata_MOD_DRV`) |
| `panel/` | Abrir `ensayo_torque.html` en Chrome o Edge |
| `ensayos/` | Registro de cada sesión |

## Diferencias con la guía

- El motor va **conectado a la MOD_DRV**, no directo a la fuente. El PWM usa avance/freno, así que el voltaje medio en bornes es ~ciclo × Vbus. Mídelo con el multímetro y escríbelo en la tabla: el estimado sale en gris.
- La velocidad de B sale del encoder, sin video.
- El torque sigue siendo masa × brazo: no depende del sensado de corriente.
- El CS no está calibrado y no es fiable bajo ~50 mA. El panel calcula un factor con 3 pulsos medidos con amperímetro.

## Protecciones del firmware

- Bloqueado: ciclo ≤ 65 % (7,8 V a 12 V) y pulso ≤ 3 s.
- Corriente > 0,6 A durante 50 ms, nFAULT, sensado perdido 100 ms o 0,6 s sin panel: motor suelto.
- En vacío o subiendo una pesa, si el eje no gira: se detiene entre 0,6 y 1 s.
- Subir la pesa termina en freno; el freno no garantiza sostenerla.
- La temperatura **no se mide**: en C hay que anotarla; el panel corta con 60 °C o 90 s sin lectura.

No minimices la pestaña durante un ensayo: el navegador frena los mensajes y la placa se detiene (`SIN_PANEL`).

## Estado

Compilado con ESP-IDF 5.4.4. Panel probado con el ESP32 simulado. **Sin probar en la placa ni con un motor.**
