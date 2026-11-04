# Índice de Entregables Oficiales · Proyecto Final

En este directorio se compilan los reportes técnicos, especificaciones y documentos formales del proyecto de robótica y visión computacional con el TurtleBot 4.

---

## 📑 Documentos Disponibles

### 1. [Informe Técnico: Arquitectura de Navegación Reactiva vs. Nav2/SLAM](informe_arquitectura_navegacion.md)
- **Tipo:** Informe Técnico de Arquitectura y Rendimiento.
- **Fecha:** 03/11/2026.
- **Contenido:**
  - Comparativa integral de rendimiento computacional (CPU, RAM, latencia DDS).
  - Justificación de por qué la arquitectura reactiva (VFH-lite + migas + ICP 2D) es superior a Nav2/MPPI para circuitos ciegos en la Raspberry Pi 4.
  - Análisis de ventajas, desventajas y casos de uso en competencia.
  - Implementación de estructuras de datos optimizadas (`SpatialKDTree2D`, `BreadcrumbGraph`).
  - Guía de despliegue a bordo (*Edge Execution*) con 0 ms de latencia.

### 2. [Informe de Estado y Madurez al 70%](informe_estado_proyecto_70.md)
- **Tipo:** Reporte Formal de Estado y Roadmap de Pruebas.
- **Fecha:** 04/11/2026.
- **Contenido:**
  - Diagnóstico exhaustivo del avance global (fijado estrictamente en 70%).
  - Mapeo de la Tarea A ($A \to B \to \text{Escaneo} \to B \to A \to \text{Anuncio}$) y Tarea B (MediaPipe + Guardián LiDAR).
  - Estrategia defensiva "Cero Colisiones" (inflado $R=26\text{ cm}$, rotación pura en frente $<32\text{ cm}$, desaceleración suave y bumper).
  - Cobertura de pruebas unitarias (62 pass / 1 skip) y plan para el 30% restante de pruebas en pista física.

---

## 📂 Próximos Entregables del Calendario
- `entregables/reporte_fase_vision_facial.pdf`: Evaluación de precisión y matriz de confusión del clasificador SFace + YuNet con galería real.
- `entregables/presentacion_final.pptx`: Diapositivas de defensa del proyecto final ante el jurado del curso.
