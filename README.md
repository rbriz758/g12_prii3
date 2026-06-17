<div align="center">
  <h1>🤖 Proyecto Eurobot: Navegación Autónoma ROS 2 (g12_prii3)</h1>

  <p>
    <strong>Sistema multi-robot autónomo basado en ROS 2 con Navegación por Máquina de Estados (FSM), Visión Artificial (ArUco) e integración de Hardware.</strong>
  </p>

  <!-- Tecnologías -->
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 2" />
  <img src="https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python" />
  <img src="https://img.shields.io/badge/OpenCV-4.5-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white" alt="OpenCV" />
  <img src="https://img.shields.io/badge/Gazebo-Simulation-FFB800?style=for-the-badge&logo=gazebo&logoColor=white" alt="Gazebo" />
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Ubuntu" />
  <br>
  <img src="https://img.shields.io/badge/TurtleBot3-Burger%20%7C%20Waffle-blue?style=for-the-badge" alt="TurtleBot3" />
  <img src="https://img.shields.io/badge/NVIDIA-JetBot-76B900?style=for-the-badge&logo=nvidia&logoColor=white" alt="JetBot" />
</div>

<br>

## 📝 Descripción del Proyecto

Este repositorio documenta el desarrollo y progresión de un proyecto de robótica para crear un **Eurobot Autónomo**, divido en sprints de desarrollo que abarcan desde el aprendizaje básico con **Turtlesim**, la implementación en el entorno de simulación **Gazebo** con control de colisiones, hasta la navegación final, integración hardware de **TurtleBot3** y **NVIDIA JetBot**, y despliegue de **Visión por Computadora (Computer Vision)** mediante **Máquinas de Estados Finitos (FSM)**.

### ✨ Características Principales:
- **Trazado de trayectorias complejas:** Capacidad de dibujar formas/números de manera parametrizable.
- **Evitación de Colisiones (Collision Avoidance):** Controladores reactivos por Lidar (LaserScan).
- **Navegación Autónoma (Nav2):** Mapeo y navegación autónoma en entornos simulados (Gazebo).
- **Computer Vision & ArUco:** Calibración de cámaras y reconocimiento de marcadores ArUco para orientación o comandos.
- **Arquitectura FSM (Finite State Machine):** Coordinación avanzada de estados de misión, navegación y control de brazo robótico (Hardware integration).

---

## 📂 Estructura del Repositorio

El proyecto se divide en módulos ROS 2 específicos para diferentes tareas y robots:

- 🐢 `g12_prii3_turtlesim`: Desarrollo inicial en el entorno 2D Turtlesim.
- 🚗 `g12_prii3_move_turtlebot`: Lógica de trazado de formas y evitación de obstáculos reactiva para el TurtleBot3 (Gazebo).
- 🚙 `g12_prii3_move_jetbot`: Adaptación y control de movimiento transferidos al hardware físico del NVIDIA JetBot.
- 🗺️ `g12_prii3_nav_turtlebot`: Sistema de navegación (Nav2), creación de mapas, y despliegue de misiones y comportamiento FSM.
- 📷 `Calibracion_camara`: Scripts Python basados en OpenCV para realizar la calibración intrínseca de la cámara y rectificación de imágenes.
- 🏗️ `F1A1`: Modelos y entornos descriptivos (SDF/Config).

---

## ⚙️ Requisitos Previos

- Sistema Operativo: **Ubuntu** (Compatible con ROS 2 Humble/Foxy)
- Instalación base de **ROS 2** configurada (`source /opt/ros/<distro>/setup.bash`)
- `colcon` instalado para la compilación del workspace.
- Paquetes de simulación **Gazebo** (`gazebo_ros_pkgs`) y **Nav2** (`navigation2`).
- Dependencias de TurtleBot3: `turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations`.

---

## 🚀 Instalación y Compilación

1. **Clonar el repositorio:**
   ```bash
   git clone https://github.com/rbriz758/g12_prii3.git
   cd g12_prii3
   ```

2. **Compilar el Workspace:**
   ```bash
   colcon build
   ```

3. **Configurar el entorno:**
   ```bash
   source install/setup.bash
   export TURTLEBOT3_MODEL=burger  # o 'waffle' dependiendo del launch
   ```

---

## 🛠️ Modos de Ejecución (Sprints y Fases)

### 1. Turtlesim (Comportamiento Básico)
Lanza el nodo inicial de Turtlesim capaz de trazar trayectorias programadas.
```bash
ros2 launch g12_prii3_turtlesim mover_tortuga.launch.py
```
> **Servicios disponibles:** Parar, Reanudar y Reiniciar la simulación a través de peticiones `std_srvs/srv/Trigger`.

### 2. TurtleBot3 Move y Evitación de Obstáculos (Gazebo)
Simulación del comportamiento de movimiento y sensores Lidar del Turtlebot.
- **Solo trazado:**
  ```bash
  ros2 launch g12_prii3_move_turtlebot draw_number_only.launch.py
  ```
- **Trazado + Supervisión de Colisión:** (Detecta objetos en su camino para frenar)
  ```bash
  ros2 launch g12_prii3_move_turtlebot draw_and_collision.launch.py
  ```
- **Trazado + Controlador Reactivo:** (Capaz de esquivar el obstáculo)
  ```bash
  ros2 launch g12_prii3_move_turtlebot draw_and_obstacle.launch.py
  ```

### 3. Navegación Avanzada, FSM y CV (Sprint Final)
Integra todo el proyecto: Entorno complejo F1L3, pila Nav2 y misiones predefinidas guiadas por lectura de marcas **ArUco**.

*Nota: Para utilizar ArUcos, colócalos en la simulación Gazebo mediante la pestaña "Insert".*

**A) Simulador Base y Navegación:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch g12_prii3_nav_turtlebot f1l3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=src/g12_prii3_nav_turtlebot/maps/map.yaml
```

**B) Ejecución de Misiones (Launchs completos):**
- **Piloto Automático + Nav2 + Entorno:**
  ```bash
  ros2 launch g12_prii3_nav_turtlebot project_launch.launch.py
  ```
- **Trayectoria Predefinida + Entorno:**
  ```bash
  ros2 launch g12_prii3_nav_turtlebot predefined_launch.launch.py
  ```

### 4. NVIDIA JetBot (Despliegue Hardware Físico)
Ejecución en remoto a través de la arquitectura del JetBot.
```bash
# En el JetBot (Terminal 1)
ros2 launch jetbot_pro_ros2 jetbot.py

# En el JetBot (Terminal 2)
source install/setup.bash
ros2 launch g12_prii3_move_jetbot draw_number.launch.py
```

---

## 🤝 Autor / Contribuciones
Proyecto universitario para la asignatura Proyecto III. Desarrollado y mantenido por **[rbriz758](https://github.com/rbriz758)** y colaboradores.
