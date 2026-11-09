---
tipo: reunion
fecha: 2026-11-08
fuente: https://claude.ai/artifact/KYwkWDKfsEjGbrRtk8eovn
aplicado: 2026-11-09
tags:
  - decisiones
  - cambios
---

# Cambios acordados – Tesis hexápodos (2026-11-08)

Notas de la llamada de voz para actualizar los documentos. Estado de cada cambio al 2026-11-09:

| # | Cambio | Estado | Dónde |
|---|---|---|---|
| 1 | U1: LiPo **3S** (11,1 V), la roja prestada por Marco | ✅ Aplicado | [[arquitectura-electronica-hexapodos]], [[bateria-lipo-3s]] |
| 1 | U2: batería de **~14,4 V (4S)** | ✅ Aplicado; ⏳ falta el modelo y la química | [[arquitectura-electronica-hexapodos]], [[Modulo de control de extremidad - Registro del proyecto]] |
| 1 | TVS SMAJ15A de la MOD_DRV (~16,7 V): mantener el MP1584 de 12 V entre la batería y las patas | ✅ Anotado | ídem |
| 1 | **Se descartan las baterías 2S** | ✅ Aplicado en las notas; ⏳ falta en Lucidchart/TikZ | — |
| 2 | Nombre único del motor: "micromotor Pololu HPCB 12 V 100:1 (formato N20)" | ✅ En las notas nuevas | — |
| 3 | **Bus CAN como enlace principal**; Wi-Fi o ESP-NOW como respaldo (propuesta a futuro) | ✅ Aplicado | [[justificacion-esp32-por-pata]] |
| 4 | Actualizar Lucidchart y las figuras TikZ | ⏳ Pendiente (manual) | — |
| 5 | Problemática y aplicación | ✅ Nota nueva | [[problematica-y-aplicacion]] |
| 6 | Gases a mapear y límites del DS 024-2016-EM | ✅ Nota nueva (falta verificar el Anexo 15) | [[gases-a-monitorear]] |
| 7 | Nivel de cooperación (Farinelli et al., 2004) | ✅ Nota nueva | [[nivel-de-cooperacion]] |
| 8–9 | Normas y clasificación por tamaño | ✅ Nota nueva, verificada parcialmente | [[normas-y-reglamentos]] |

## Pendiente
- [ ] Confirmar la especificación exacta de la batería de la U2 (Alfredo, 2026-11-10).
- [ ] Actualizar Lucidchart y las figuras TikZ.
- [ ] Elegir los gases del alcance.
- [ ] Definir la tarea cooperativa.
- [ ] Pesar y medir la U1 y la U2.
