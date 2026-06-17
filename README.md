# Proyecto III - Robótica Móvil (Grupo 12)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv)

Este repositorio alberga el código fuente y la documentación del proyecto anual de la asignatura **Proyecto III - Robótica Móvil**. El trabajo abarca desde la simulación cinemática básica hasta el despliegue de un sistema robótico autónomo distribuido, culminando en un prototipo físico para la competición **Eurobot 2026**.

## Descripción General

El objetivo de este repositorio es documentar el progreso a lo largo de varios *sprints* de desarrollo. Iniciando con simuladores 2D (`turtlesim`), pasando por simuladores 3D (`Gazebo` con TurtleBot3) y finalizando con la integración en hardware real (Nvidia JetBot y una plataforma diferencial personalizada con brazo robótico).

El proyecto integra:
- **Navegación Autónoma:** Sistemas reactivos, seguimiento de trayectorias y uso del stack de navegación Nav2.
- **Visión Artificial (OpenCV):** Detección de marcadores ArUco, seguimiento de líneas (siguelíneas cerrado por visión) y detección de color.
- **Arquitecturas de Control:** Máquinas de Estados Finitos (FSM) y orquestación de nodos bajo **ROS 2**.
- **Manipulación Física:** Control de brazos robóticos y actuadores neumáticos a través de puentes seriales (Arduino).

---

## Evolución del Proyecto (Sprints)

El repositorio está estructurado en distintos paquetes de ROS 2 que corresponden a las fases de aprendizaje y desarrollo de la asignatura:

### Sprint 1: Introducción a ROS 2 (`g12_prii3_turtlesim`)
Primer acercamiento a la creación de nodos, *topics* y *services*.
- **Objetivo:** Dibujar el número "12" (número del grupo) utilizando la tortuga virtual.
- **Características:** Implementación de servicios en tiempo real para controlar la ejecución (`/detener_dibujo`, `/reanudar_dibujo`, `/reiniciar_dibujo`).

### Sprint 2: Control Cinemático y Reactivo (`g12_prii3_move_turtlebot`, `g12_prii3_move_jetbot`)
Traslado de la lógica de trayectorias a simuladores 3D y a un robot real.
- **Objetivo:** Ejecutar la misma trayectoria ("12") pero añadiendo percepciones del entorno.
- **Características:** 
  - Trazado base en mundo vacío Gazebo (`draw_number_only.launch.py`).
  - Supervisor de colisiones (`draw_and_collision.launch.py`).
  - Controlador reactivo para esquivar obstáculos imprevistos (`draw_and_obstacle.launch.py`).
  - Implementación replicada y probada físicamente en el robot **JetBot**.

### Sprint 3: Navegación y Planificación (`g12_prii3_nav_turtlebot`)
Implementación de navegación autónoma avanzada usando **Nav2** en un entorno simulado (Mundo Gazebo F1L3).
- **Objetivo:** Navegar del punto de inicio a zonas objetivo tomando decisiones.
- **Características:** 
  - Piloto automático basado en la lectura de marcadores **ArUco** insertados en el entorno (determinan la ruta o la puerta a cruzar).
  - Generación de trayectorias predefinidas.
  - Orquestación del sistema mediante un archivo de lanzamiento integral (`project_launch.launch.py`).

### Calibración de Cámara (`Calibracion_camara`)
Módulo independiente para asegurar la precisión de los algoritmos de visión.
- Scripts en Python con OpenCV (`calibracion.py`, `rectificar_tablero.py`) para calcular los parámetros intrínsecos de la lente, obtener la matriz de la cámara y eliminar la distorsión de las imágenes reales para operaciones métricas de precisión.

### Sprint Final: Eurobot 2026 (Plataforma Autónoma de Manipulación)
El hito definitivo del proyecto. Unifica los conocimientos previos en una plataforma móvil diferencial real equipada con cámara V4L2, brazo robótico y ventosa neumática controlada por una **FSM**.

**Misión implementada:**
1. Navegación guiada por **siguelíneas** (visión BGR-HSV) inmune a ruidos visuales o decoraciones del circuito.
2. Detección y alineamiento dinámico basado en el área de los marcadores ArUco.
3. Maniobra de **manipulación y agarre** de la pieza con el brazo robótico y encendido de ventosa neumática.
4. Identificación visual de cruces en el trazado (intersecciones en T) y zonas de almacenaje (aislamiento de color verde).
5. Descarga de la pieza y finalización segura de la misión.

---

## Stack Tecnológico

- **Middleware:** ROS 2 (Humble / Iron)
- **Lenguaje:** Python 3.10
- **Simulación:** Gazebo Classic (TurtleBot3 Model)
- **Visión y Percepción:** OpenCV (cv2) y cv-bridge
- **Hardware Integrado (Fase Final):** 
  - Plataforma Nvidia JetBot
  - Microcontroladores Arduino (Puente serial `/dev/ttyACM0`)
  - Brazo Robótico Serial (`/dev/ttyUSB0`, `/dev/ttyUSB1`)
  - Cámara V4L2

---

## Instalación y Configuración

**Requisitos Previos:**
- SO: Ubuntu 22.04
- ROS 2 instalado y debidamente configurado.
- `colcon` para la construcción del workspace.

**Pasos de Instalación:**
```bash
# 1. Clonar el repositorio
git clone https://github.com/rbriz758/g12_prii3.git
cd g12_prii3

# 2. Compilar el workspace completo
colcon build

# 3. Cargar las variables de entorno generadas
source install/setup.bash
```

> **Nota para Simulaciones:** Asegúrate de exportar el modelo del TurtleBot3 antes de lanzar los entornos de Gazebo. Puedes hacerlo con `export TURTLEBOT3_MODEL=waffle` (o `burger`).

---

## Instrucciones de Uso Rápido

A continuación se muestran ejemplos directos de cómo lanzar los distintos apartados del proyecto:

### 1. Turtlesim (Servicios)
```bash
ros2 launch g12_prii3_turtlesim mover_tortuga.launch.py
```
*(Puedes usar el comando `ros2 service call /detener_dibujo std_srvs/srv/Trigger` o `/reanudar_dibujo` para pausar/reanudar el trazado).*

### 2. Turtlebot (Navegación reactiva ante obstáculos)
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch g12_prii3_move_turtlebot draw_and_obstacle.launch.py
```
*(Para probar los sensores, se debe colocar un objeto en forma de cubo dentro del trazado en Gazebo).*

### 3. Navegación Nav2 con Lógica ArUco
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch g12_prii3_nav_turtlebot project_launch.launch.py
```

### 4. Demostración Física Eurobot (Hardware)
```bash
# 1. Otorgar permisos a los controladores físicos y linkear puertos seriales
sudo chmod 777 /dev/ttyACM0 /dev/ttyUSB0 /dev/ttyUSB1
sudo ln -sf /dev/ttyUSB0 /dev/ttyUSB1

# 2. Asegurarse de tener el entorno cargado (Chasis, Brazo y Proyecto)
source ~/robot_ws/install/setup.bash
source ~/brazo_ws/install/setup.bash
source install/setup.bash

# 3. Lanzar la orquestación integral (FSM + Hardware Drivers)
ros2 launch sprint8_eurobot sprint8_launch.py
```
*Una vez lanzado, espera el mensaje de inicialización en la terminal, verifica las ventanas de debug visual (OpenCV), posiciona el robot y **presiona ENTER** en la terminal principal para dar comienzo a la demostración autónoma.*

---

## Acerca del Equipo

> **Equipo de Desarrollo:** Grupo 12 ([rbriz758/g12_prii3](https://github.com/rbriz758/g12_prii3))  
> **Contexto:** Trabajo de la asignatura anual centrado en el diseño algorítmico, simulación en Gazebo e integración física final de un sistema robótico distribuido bajo ROS 2.  
> **Áreas de Trabajo:** Visión artificial, Cinemática diferencial, Control embebido, Máquinas de estados, Control reactivo.
