---
tipo: fuente
clave-ieee: "[24]"
autores: ["Wang, Z.", "Chen, Z.", "Zhang, J."]
anio: 2012
titulo-completo: "Servo Motor Position Control Based on DSP"
venue-doi: "En: Mechanical Engineering and Technology, Advances in Intelligent and Soft Computing, Springer. DOI candidato: 10.1007/978-3-642-27329-2_57 (verificar)"
tags:
  - fuente
  - capitulo/metodologia
  - capitulo/resultados
---

# [24] Wang et al. (2012) — Servo Motor Position Control Based on DSP

## 📝 Resumen
El artículo presenta un sistema de control de posición para un motor DC servo de imán permanente, basado en DSP. El sistema de control se organiza en **tres lazos anidados: posición, velocidad y corriente**, cada uno con un controlador PID diseñado y ajustado según señales de retroalimentación medidas por un sensor de corriente de efecto Hall y un encoder óptico.

## 💡 Hallazgos / argumentos clave
- Arquitectura de control en cascada de 3 lazos (posición → velocidad → corriente) implementada sobre DSP, con PID independiente por lazo.
- Retroalimentación de corriente vía sensor Hall y de posición/velocidad vía encoder óptico — misma lógica de sensado que ACS712 + encoder en este proyecto.

## 🛠 Cómo lo uso en mi tesis
- Fuente principal que respalda la arquitectura de **control en cascada de tres lazos (corriente-velocidad-posición)** sobre el motor DC N20 ([[03-control]]), y el uso de ACS712 como sensor de corriente análogo al sensor Hall del paper.
- Referencia metodológica para el diseño de los controladores PID por lazo en Metodología/Resultados.

## 🔗 Relacionado
- Capítulos: [[metodologia]], [[resultados]]
- Otras fuentes: —
