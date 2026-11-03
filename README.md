# Proyecto Visión 67 🤖👁️ · TurtleBot 4 Lite

Repositorio oficial de avances, bitácora de desarrollo, arquitectura y entregables del proyecto final de robótica móvil y visión por computador.

- **Plataforma:** TurtleBot 4 Lite (iRobot Create 3 + RPLIDAR A1 + OAK-D Lite + Raspberry Pi 4)
- **Framework:** ROS 2 Jazzy Jalisco (Ubuntu 24.04 LTS)
- **Fecha de Entrega Final:** 22 de Noviembre de 2026

---

## 👥 Equipo y Colaboradores
- **Líder / Desarrollador:** Mando ([@Tilinpro19](https://github.com/Tilinpro19))
- **Equipo de Proyecto:** *(Configurar invitaciones en [Ajustes de Acceso](https://github.com/Tilinpro19/proyecto-vision-67/settings/access))*

---

## 🎯 Objetivo y Tareas de la Competencia

El proyecto consiste en resolver dos misiones de alta precisión en un circuito con obstáculos ciegos (revelados el mismo día del examen):

1. **Tarea A (Navegación Autónoma y Reconocimiento Facial):**
   - Navegación $A \to B$ evadiendo obstáculos y paredes desconocidas.
   - Escaneo facial en $B$ con la cámara OAK-D Lite (detección YuNet + embeddings SFace).
   - Retorno autónomo $B \to A$ guiándose por el rastro de migas y corrigiendo deriva de odometría con **ICP 2D**.
   - Anuncio verbal del nombre de la persona en $A$. Cero intervención humana.
2. **Tarea B (Teleoperación Gestual y Reconocimiento Facial):**
   - Mismo recorrido y objetivos, pero controlado 100% mediante gestos de manos con MediaPipe Hand Landmarker (prohibido el uso de teclado, mouse o joystick).

---

## 📂 Estructura del Repositorio

```text
proyecto-vision-67/
├── README.md               # Portada principal, estado y enlaces a documentación
├── bitacora/               # Registro cronológico semanal de actividades y avances
│   ├── plantilla_avance.md # Plantilla oficial para registrar nuevas sesiones
│   ├── semana-01.md        # Semana 1: Configuración inicial y separación de repos
│   └── semana-02.md        # Semana 2: Optimización de navegación, reloj y edge deployment
├── docs/                   # Documentación técnica de referencia y manuales
│   ├── lineamientos_competencia.md      # Reglas oficiales, rúbrica de puntaje y penalizaciones
│   ├── arquitectura_sistema.md          # Diagramas de flujo, máquinas de estados y módulos
│   ├── manual_operacion_laboratorio.md  # Guía de conexión Ethernet, reloj y ejecución on-board
│   └── resolucion_problemas_tecnicos.md # Historial técnico de fallas resueltas (reloj, saturación CPU)
├── entregables/            # Informes técnicos, reportes PDF y presentaciones
│   ├── README.md           # Índice de entregables
│   └── informe_arquitectura_navegacion.md # Informe técnico comparativo VFH vs Nav2/SLAM
└── evidencias/             # Capturas de sensores, registros de pruebas y fotos
    └── README.md           # Bitácora de evidencias y pruebas unitarias
```

---

## 📌 Enlace al Repositorio de Código Operativo
El código fuente de ejecución en el robot, tests unitarios y herramientas de bajo nivel se mantienen en:
- Repositorio técnico: [turtlebot67 (turtleclaude4)](https://github.com/Tilinpro19/turtlebot67)

---

## 🚀 Estado de Avances y Bitácora

| Periodo | Hito / Avance Técnico | Documento | Estado |
|---|---|---|---|
| **Semana 01** | Separación de repositorios y estructuración del espacio de trabajo | [`bitacora/semana-01.md`](bitacora/semana-01.md) | ✅ Completado |
| **Semana 02** | Sincronización de reloj (0.08 s), cinemática suave, `SpatialKDTree2D`, `BreadcrumbGraph` y despliegue a bordo | [`bitacora/semana-02.md`](bitacora/semana-02.md) | ✅ Completado |

---

## 📑 Documentación Técnica Destacada

- 📖 **[Lineamientos y Rúbrica de la Competencia](docs/lineamientos_competencia.md):** Fórmulas de evaluación ($4.5 \cdot \frac{t_{\text{max}} - t}{t_{\text{max}} - t_{\text{min}}}$), penalidades y reglas de la pista.
- 📐 **[Arquitectura Integral del Sistema](docs/arquitectura_sistema.md):** Explicación del coordinador `mision.py`, el piloto `navegacion.py`, visión `facial.py` y teleoperación `gestos_teleop.py`.
- 🛠️ **[Manual de Operación en Laboratorio](docs/manual_operacion_laboratorio.md):** Guía rápida para conexión Ethernet (`192.168.185.3`), sincronización de reloj y comandos de ejecución on-board.
- 🩺 **[Registro de Problemas Resueltos](docs/resolucion_problemas_tecnicos.md):** Análisis a fondo de la saturación de la Raspberry Pi con Nav2/MPPI y el desfase de 20.9 h del Create 3.
- 📄 **[Informe Técnico Comparativo](entregables/informe_arquitectura_navegacion.md):** Comparativa detallada de rendimiento: Navegación Reactiva vs. SLAM/Nav2 tradicional.
