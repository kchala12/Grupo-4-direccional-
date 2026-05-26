# 💻 src — Código Fuente

> Módulos de software del **Robot Móvil Autónomo para Transporte Inteligente de Materiales**  
> Ingeniería Mecatrónica · Universidad Tecnológica de Pereira · 2026

---

## 🗂️ Contenido de la Carpeta

### 📁 .vscode
Configuración del entorno de desarrollo Visual Studio Code. Contiene ajustes del intérprete de Python, tareas de ejecución remota y configuración de depuración para trabajar directamente sobre la Raspberry Pi 5 y ESP-32.

### 📁 pruebas camara imx219
Scripts y recursos utilizados durante las pruebas de integración y calibración de la cámara **IMX219**. Incluye pruebas de captura, ajuste de resolución, enfoque y validación de la detección de códigos QR en distintas condiciones de iluminación.

### 📁 pruebas pasadas
Versiones anteriores de scripts y experimentos realizados durante el desarrollo. Sirve como historial de iteraciones previas del sistema de navegación, control de motores y detección visual, incluyendo las actualizaciones de los puntos de descarga.

---

## 📑 Archivos Principales

| Archivo | Descripción |
|---|---|
| `principal16.py` | Script principal del sistema. Ejecuta el ciclo autónomo completo: detección QR, navegación y descarga en las estaciones. **Punto de entrada del robot.** |
| `camara_detector27.py` | Módulo de visión artificial. Gestiona la captura de imágenes con la cámara IMX219 y la detección/decodificación de códigos QR en tiempo real. |
| `motor_control7.py` | Módulo de control de motores. Gestiona la velocidad, dirección y parada del sistema de tracción del robot según las instrucciones de navegación. |


---

## 🚀 Ejecución

### Requisitos previos

- Raspberry Pi 5 con Raspberry Pi OS (64-bit)
- Python 3.10+
- Cámara IMX219 conectada y habilitada (`raspi-config`)

### Instalar dependencias

```bash
pip install -r requirements.txt
```

### Calibrar encoders (primera vez)

```bash
python calibrar_encoders.py
```

### Ejecutar el sistema principal

```bash
python principal16.py
```

---

## 🧩 Arquitectura del Software

```
principal16.py
    ├── camara_detector27.py   →  Detección de códigos QR
    └── motor_control7.py      →  Control de motores y tracción
```

---

## 📌 Notas

- Los archivos están nombrados con versión numérica (ej. `principal16.py`) para llevar trazabilidad de iteraciones.
- Se recomienda ejecutar siempre desde la raíz de la carpeta `src/`.
- Para pruebas individuales de cada módulo, revisar la carpeta `pruebas pasadas/`.
