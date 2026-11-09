---
tipo: fuente
tags:
  - control
  - fuente
---

# Investigación: control de corriente (PI/PID) para motores DC

Búsqueda pedida: literatura sobre control de corriente (PI/PID) en motores DC, priorizando artículos con **enfoque didáctico**. Ninguna de estas tiene todavía un número IEEE `[n]` asignado en tu bibliografía — son candidatas. Si adoptas alguna, créale su propia nota con `templates/fuente.md` en `05-documentacion/estado-del-arte/`, numerada según el orden real en que aparezca en tu documento, y enlázala desde acá.

## 1. Enfoque didáctico (lo que pediste explícitamente)

**Kamarudin, M. N.; Rozali, S. M. (2008).** "Simulink Implementation of Digital Cascade Control DC Motor Model – A didactic approach." *2008 IEEE 2nd International Power and Energy Conference (PECon)*, pp. 1043-1048.
- Desarrollo paso a paso pensado explícitamente para que estudiantes de ingeniería eléctrica entiendan el control en cascada. Compara un PI convencional contra un PI auto-sintonizado con lógica difusa.
- **Por qué te sirve:** es el candidato más directo a tu pedido — misma estructura de cascada que usas (corriente dentro, velocidad/posición afuera), explicado de forma pedagógica.
- [ResearchGate](https://www.researchgate.net/publication/224375985_Simulink_Implementation_of_Digital_Cascade_Control_DC_Motor_Model_-_A_didactic_approach) · [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/4762627/) · [Academia.edu (PDF)](https://www.academia.edu/3788209/Simulink_Implementation_of_Digital_Cascade_Control_DC_Motor_Model_A_didactic_approach)

## 2. Control de corriente — artículos específicos

**Ramesh, K.; Ayyar, K.; Nirmalkumar, A.; Gurusamy, G. (2010).** "Design of Current Controller for Two Quadrant DC Motor Drive by Using Model Order Reduction Technique." arXiv:1002.1683.
- Diseño de un controlador de corriente para motor DC de dos cuadrantes, reduciendo el orden del modelo de planta para simplificar el ajuste del PI/PID.
- [arXiv (PDF)](https://arxiv.org/pdf/1002.1683)

**(2023).** "Design and Development of Cascaded Current Control in DC Motor Variable Speed Drive using dSPACE." *Journal of Physics: Conference Series*, vol. 2641, 8th International Conference on Man Machine Systems 2023.
- Cascada con PI en el lazo de velocidad y controlador de histéresis en el lazo de corriente; validado en hardware real (dSPACE 1104 + puente asimétrico + gate driver). Relevante porque no se queda en simulación, como tampoco debería quedarse tu proyecto.
- ⚠️ Autores no confirmados en el snippet de búsqueda — verificar antes de citar.
- [ResearchGate](https://www.researchgate.net/publication/392658066_Design_and_Development_of_Cascaded_Current_Control_in_DC_Motor_Variable_Speed_Drive_using_dSPACE) · [ADS/Harvard](https://ui.adsabs.harvard.edu/abs/2023JPhCS2641a2029A/abstract)

**"Practical implementation of cascade control approach with PI controllers to control the speed of a permanent magnet DC motor."** ResearchGate.
- Cascada PI-PI (corriente interna, velocidad externa) para un motor DC de imán permanente — arquitectura muy cercana a la de tu N20.
- ⚠️ Autor y año no confirmados — verificar en la publicación completa antes de citar.
- [ResearchGate](https://www.researchgate.net/publication/343972053_Practical_implementation_of_cascade_control_approach_with_PI_controllers_to_control_the_speed_of_a_permanent_magnet_DC_motor)

**"Modeling and Instability of Average Current Control."** arXiv:1210.1959.
- Más teórico: analiza condiciones de inestabilidad (subarmónica) del control de corriente promedio (average current-mode control). Útil si en Resultados necesitas justificar por qué tu lazo de corriente es estable y no solo que "funciona".
- [arXiv (PDF)](https://arxiv.org/pdf/1210.1959)

## 3. Control en cascada corriente-velocidad-posición (contexto directo de tu arquitectura)

Ya tienes en el estado del arte a [[24-wang-servo-motor-position-control-dsp]] (Wang et al. 2012) con la misma arquitectura de 3 lazos. Estas dos amplían esa línea:
- Kamarudin & Rozali (2008) — sección 1.
- "Design and Development of Cascaded Current Control..." (2023) — sección 2.

## 4. Recursos educativos de referencia (tutoriales / application notes, no artículos revisados por pares)

**University of Michigan — Control Tutorials for MATLAB and Simulink.**
- ["PI Control of DC Motor Speed"](https://ctms.engin.umich.edu/CTMS/index.php?aux=Activities_DCmotorB)
- ["Motor Speed: PID Controller Design"](https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=ControlPID)
- ["Motor Position: PID Controller Design"](https://ctms.engin.umich.edu/CTMS/index.php?example=MotorPosition&section=ControlPID)
- No es un paper citable en IEEE de forma típica, pero es el tutorial didáctico de referencia más usado en cursos de control para sintonizar PI/PID por ubicación de polos en lazo cerrado — útil como apoyo metodológico, no como fuente de estado del arte.

**Renesas — Application Note.** ["Position and Speed Control of a DC Motor using Analog PID"](https://www.renesas.com/en/document/apn/cm-250-position-and-speed-control-dc-motor-using-analog-pid-controller)
- Nota de aplicación industrial con enfoque práctico, cercana a lo que harás con ESP32 + DRV8876.

## 5. En español (citables directamente, útiles para Metodología)

- **Repositorio UCSM (Perú).** ["Modelamiento y control de velocidad de motor DC mediante algoritmo PID basado en reglas de Ziegler-Nichols"](https://repositorio.ucsm.edu.pe/items/76b485d6-5a46-4a35-9ea8-24c7b315569e) — sintonización PID con Ziegler-Nichols sobre ATmega328, misma metodología de sintonización que probablemente uses.
- **Repositorio UTP (Colombia).** ["Control PID para el control de velocidad de un motor DC"](https://repositorio.utp.edu.co/server/api/core/bitstreams/1a4fb4e4-bc53-4e89-ab3b-cd028b2e8da6/content) — diseño PID desde modelo idealizado, comparando P, PI y PID, con ubicación de raíces.

## 🔗 Relacionado
- [[24-wang-servo-motor-position-control-dsp]]
- [[control-motores-esp32-3x-drv8876]]
- [[TESIS]]
