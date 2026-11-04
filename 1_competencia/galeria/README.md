# Galería de personas

Una carpeta por persona; el nombre de la carpeta es lo que dice el robot (`_` se lee como espacio):

```
galeria/
  Juan_Perez/  foto1.jpg  foto2.jpg ...
  Maria/       a.png ...
```

- Varias fotos por persona ayudan (distinta luz/ángulo). Si dan solo la foto impresa, captúrala con la cámara del robot: `python3 facial.py --capturar Juan_Perez --n 15`.
- `python3 facial.py --probar` revisa la galería (deja-uno-fuera) y avisa fotos sin cara.
- Los embeddings se guardan en `.cache.npz`; se recalculan solos si cambian las fotos.
