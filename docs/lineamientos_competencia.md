# Lineamientos y Reglas de la Competencia · Proyecto Final

- **Fecha de Entrega:** 22 de Noviembre de 2026
- **Plataforma Oficial:** TurtleBot 4 Lite (Base móvil iRobot Create 3 + RPLIDAR A1 + Cámara Estéreo OAK-D Lite + Raspberry Pi 4)
- **Condición de Evaluación:** Entorno no mapeado previamente. El circuito de obstáculos y las fotos de las personas se revelan el mismo día de la competencia.

---

## 🎯 Descripción de las Tareas

### Tarea A · Navegación Autónoma y Reconocimiento Facial
1. **Salida:** El robot inicia parado dentro de la zona delimitada como $A$.
2. **Trayecto de Ida ($A \to B$):** Navega de forma completamente autónoma evadiendo obstáculos estáticos y paredes hasta alcanzar la zona $B$.
3. **Escaneo y Clasificación:** En $B$, utiliza la cámara OAK-D Lite para escanear la fotografía de la persona colocada en el punto de destino, extrayendo el embedding facial y comparándolo contra la galería offline pre-cargada.
4. **Trayecto de Regreso ($B \to A$):** Emprende el retorno a la posición inicial $A$ guiándose por el camino recorrido y corrigiendo la deriva odometría.
5. **Anuncio:** Al detenerse en $A$, el robot debe anunciar verbalmente el nombre correcto de la persona identificada.
6. **Restricción:** Cero intervención humana durante todo el recorrido.

### Tarea B · Teleoperación por Visión/Gestos y Reconocimiento Facial
1. **Salida y Control:** El robot inicia en $A$, pero es guiado mediante visión por computadora a través de gestos corporales y de manos detectados por cámara (MediaPipe Hand Landmarker).
2. **Regla de Oro:** **Prohibido el uso de teclado, mouse, joystick o comandos remotos por terminal** durante el recorrido. El operador solo interactúa frente a la cámara de control.
3. **Misión en $B$ y Regreso a $A$:** En $B$, el operador activa el escaneo gestualmente o por parada; el robot procesa la foto y se guía de regreso a $A$ donde anuncia el nombre de la persona.

---

## 📊 Criterios de Evaluación y Rúbrica de Puntaje

El puntaje asignado a cada tarea sigue la fórmula relativa:

$$P = 4.5 \cdot \frac{t_{\text{max}} - t}{t_{\text{max}} - t_{\text{min}}}$$

Donde:
- $t$: Tiempo total de la misión, sumando las penalidades acumuladas.
- $t_{\text{max}}$ y $t_{\text{min}}$: Tiempos máximos y mínimos registrados entre todos los equipos de la competencia.
- Si el robot no logra regresar a $A$, o si $t > t_{\text{max}}$, el puntaje del intento es **0**.
- Se permiten hasta **3 intentos por tarea**, computándose el mejor puntaje obtenido.

### Penalizaciones
El tiempo registrado $t$ es penalizado severamente ante:
1. **Choque contra obstáculo:** Suma segundos de penalidad por cada evento de contacto.
2. **Choque o roce contra pared:** Penalización añadida.
3. **Identificación errónea de la persona:** Penalización en tiempo o anulación del componente de reconocimiento.
4. **Abandono o parada de emergencia:** Intento fallido.

> [!IMPORTANT]
> **Filosofía de diseño ganadora:** Priorizar la seguridad y precisión sobre la velocidad extrema. Completar el recorrido sin colisiones y con reconocimiento correcto garantiza una calificación alta; arriesgar velocidad con choques degrada rápidamente el puntaje por penalizaciones.
