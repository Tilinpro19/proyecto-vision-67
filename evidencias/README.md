# Registro de Evidencias · Proyecto Final

En este directorio se almacenan capturas de pantalla de terminales, registros de sensores, diagramas y fotografías del TurtleBot 4 durante las pruebas de laboratorio.

---

## 📷 Evidencias Registradas

### Sesión 02 (03/11/2026) · Diagnóstico de Sensores y Despliegue On-Board
- **Salud del Hardware:**
  - `throttled=0x0` (sin caídas de voltaje ni sobrecalentamiento).
  - Temperatura CPU Raspberry Pi 4: 55.5 °C estable.
  - Cámara OAK-D Lite: Enlace USB 3.0 SuperSpeed activo (5000 Mbps).
  - Batería Create 3: ~78% (15.2 V).
- **Frecuencias de Sensores Validadas en Vivo:**
  - LiDAR (`/scan`): **7.6 Hz** (perfil de 360° limpio).
  - Odometría (`/odom`): **17.2 Hz** (encoders Create 3).
  - Cámara RGB (`/oakd/rgb/preview/image_raw/compressed`): **24.6 FPS**.
  - IMU (`/imu`): **32.8 Hz**.
- **Sincronización del Reloj:**
  - Desfase Create 3 ↔ Raspberry Pi reducido de **+75,368 s (20.9 h)** a **0.08 s** mediante llamada a `http://192.168.186.2/api/restart-ntpd`.
- **Suite de Pruebas Unitarias:**
  - **62 pruebas pasadas exitosamente (100% de cobertura de evasión, cinemática, gestos y visión)**.
