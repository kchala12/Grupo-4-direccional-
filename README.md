# Grupo-4-direccional

> **Robot Móvil Autónomo para Transporte Inteligente de Materiales**  
> Proyecto — Ingeniería Mecatrónica · Universidad Tecnológica de Pereira · 2026

---

## Descripción

Diseño y desarrollo de un **Vehículo Guiado Autónomo (AGV)** con visión artificial, capaz de transportar y descargar una carga de hasta **1 kg** a través de tres estaciones predefinidas, simulando operaciones logísticas en entornos industriales controlados.

El sistema utiliza **códigos QR** como referencias de navegación y una cámara **IMX219** para su detección en tiempo real, todo procesado sobre una **Raspberry Pi 5**.

---

## Objetivo

Desarrollar un robot funcional que complete de forma autónoma un ciclo de recolección y distribución de carga hacia tres estaciones destino con traccion diferencial, **sin intervención humana**, en un plazo de **16 semanas**.


---

##  Estructura del Repositorio

```
Grupo-4-direccional/
│
├── Documentacion/       # Matriz de documentos, informes y entregables
├── Electronica/         # Esquemas eléctricos, PCBs y firmware
├── Mecanica/            # Diseños CAD, planos y ensamblajes
├── src/                 # Código fuente principal (visión, navegación, control)
├── .gitignore
├── LICENSE
└── README.md
```
### 📁 [Documentacion](https://github.com/kchala12/Grupo-4-direccional-/tree/main/Documentacíon)
Contiene toda la documentación formal del proyecto: matriz de documentos, cronogramas, actas de reunión, informes de avance semanales y el informe final. También incluye presentaciones y entregables académicos requeridos por la universidad.
 
### 📁 [Electronica](https://github.com/kchala12/Grupo-4-direccional-/tree/main/Electronica)
Aquí se encuentran los esquemas eléctricos del sistema, diagramas de conexión de los componentes, diseños de PCB, selección y hoja de datos de los actuadores y drivers utilizados. También incluye el firmware de los microcontroladores encargados del control de motores y señales de los sensores.
 
### 📁 [Mecanica](https://github.com/kchala12/Grupo-4-direccional-/tree/main/Mecanica)
Contiene los diseños CAD del chasis del robot, planos técnicos de las piezas fabricadas o impresas en 3D, ensamblajes del sistema de tracción, soporte de carga y montaje de la cámara. Incluye también los archivos de simulación estructural y las memorias de cálculo de resistencia y peso.
 
### 📁 [src](https://github.com/kchala12/Grupo-4-direccional-/tree/main/src)
Código fuente principal del sistema autónomo. Incluye los módulos de visión artificial para la detección de códigos QR, algoritmos de navegación y control de trayectoria, lógica de gestión de estaciones y comunicación entre subsistemas. Todo desarrollado en Python sobre Raspberry Pi 5.
---


## 👥 Equipo

**Investigadores**
- Mateo Tabares Gil
- Federico Gómez Lotero
- Juan Sebastián Flórez Molina
- Kevin Andrés Chala González

**Director**
- Osiel Arbeláez Salazar

**Asesores**
- Angie Tatiana Rengifo Oviedo
- Carlos Andrés Rodríguez Pérez
- Edward Andrés González Ríos

---

## 🏛️ Institución

**Universidad Tecnológica de Pereira — Facultad de Tecnología — Febrero 2026**

---

## 📄 Licencia

Este proyecto está bajo la licencia especificada en el archivo [LICENSE](./LICENSE).
