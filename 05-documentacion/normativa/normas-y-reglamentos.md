---
tipo: marco-normativo
fecha: 2026-11-09
estado: verificado parcialmente
tags:
  - normativa
  - iso
  - astm
  - seguridad
---

# Normas y reglamentos aplicables

| Norma | Qué cubre | Uso en la tesis | Verificación |
|---|---|---|---|
| **ISO 8373:2021** | Vocabulario de robótica (robot de servicio, robot móvil, robot con patas) | Definir y clasificar los hexápodos | Código y año conocidos; falta revisar las definiciones exactas |
| **ISO 22166-1:2021** | Modularidad en robots de servicio, parte 1: requisitos generales. Distingue *módulo* (componente con interfaces y perfil de propiedades definidos) de *componente*. Cubre componibilidad, integrabilidad, interoperabilidad, granularidad, reutilización, seguridad y protección, e incluye una plantilla de módulo | Respaldo directo para las patas modulares de la U2 (MOD_DRV con interfaz de potencia y CAN definida) | ✅ Verificado (ISO/TC 299) |
| **ISO 13482:2014** | Seguridad de robots de cuidado personal | Referencia parcial de seguridad (no es de minería) | Código y año conocidos |
| **IEC 60079** (esquemas ATEX / IECEx) | Equipos para atmósferas explosivas | Clave con metano: declarar como **limitación** que los prototipos no son intrínsecamente seguros | Serie conocida |
| **IEC 60529** | Grado IP (polvo y agua) | Protección contra el polvo de mina | Conocida |
| **ASTM E54.09** (subcomité de *Response Robots*, con NIST) | Métodos de prueba para robots de respuesta: movilidad, maniobra, sensado, radiocomunicación, **mapeo de interiores** (WK85837), terreno de *stepfields* (WK88219) | Diseñar las pruebas de laboratorio de movilidad, mapeo y comunicación | ✅ Subcomité verificado; las normas activas (p. ej., E2826–E2830, E2991) hay que revisarlas una por una |
| **ISO 11898-1:2024 / 11898-2:2024** | Bus CAN: capa de enlace (tramas, arbitraje, errores) y capa física de alta velocidad (niveles, transceptor, terminación de 120 Ω) | Bus entre las 6 MOD_DRV y la maestra ESP32-S3 de la U2. Ver [[sustento-electronica-normas-can]] | ✅ Ediciones 2024 verificadas (ISO); revisar si hay edición 2026 de la parte 2 |
| **IPC-2221B / IPC-2152** (+ IPC-7351, IPC-6012, IPC-A-600, IPC-A-610, J-STD-001) | Diseño de PCB (pistas, separaciones, capacidad de corriente), huellas, fabricación y soldadura | Ancho de los buses de potencia de la PCBicho y la MOD_DRV. Ver [[sustento-electronica-normas-can]] | ✅ Códigos y alcance verificados |
| **DS 024-2016-EM** (Perú), modificado por el **DS 023-2017-EM** | Reglamento de Seguridad y Salud Ocupacional en Minería | O₂ ≥ 19,5 % (art. 246) y límites de exposición del Anexo 15 → umbrales de alarma. Ver [[gases-a-monitorear]] | ✅ Art. 246 verificado; ⚠️ Anexo 15 tomado de una fuente secundaria |

## Clasificación por tamaño
Ninguna norma ISO define si un robot es grande o pequeño. Hay dos formas de justificarlo:
1. **Por masa y transportabilidad:** si lo lleva una persona, como en las categorías de robots de respuesta de NIST/ASTM o de UGV (micro, mini, pequeño). *Los rangos exactos no se han verificado.*
2. **Por la tarea:** el ancho mínimo de paso del robot frente al ancho de los socavones o rejillas que debe atravesar.

- [ ] Pesar y medir la U1 y la U2 (masa, largo, ancho y alto de pie).
- [ ] Revisar las normas ASTM E2826–E2830 y E2991 y elegir 2 o 3 pruebas para el laboratorio.
- [ ] Obtener el texto oficial del Anexo 15.

> [!question] Pregunta abierta
> ¿El asesor exige alguna norma en particular, por ejemplo una NTP peruana o una de la plantilla UTEC?

## Fuentes
- ISO/TC 299, ISO 22166-1:2021: https://committee.iso.org/sites/tc299/home/projects/published/iso-22166-12021.html
- ASTM, Subcomité E54.09 on Response Robots: https://www.astm.org/jurisdiction-e5409
- NIST, Ground tests 3, Maneuvering and mobility: https://www.nist.gov/document/ground-tests-3-maneuvering-and-mobility-2022a2
- DS 023-2017-EM: https://www.ighgroup.com/NormasLegalesPDF/MODIFICACI%C3%93N%20D.S.024-2016-EM.pdf

Relacionado: [[sustento-electronica-normas-can]] · [[problematica-y-aplicacion]] · [[nivel-de-cooperacion]]
