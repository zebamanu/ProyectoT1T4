# **Proyecto de  desarrollo de Software y Hardware con ROS**

> **Proyecto T1T4: Robot Humanoide con 17 DOF controlado de forma remota** 

## **Índice**
1. [Descripción general](#descripción-general)
2. [Características](#características)
3. [Hardware](#hardware)
4. [Software](#software)
5. [Instalación](#instalación)
6. [Ejecución del proyecto](#ejecución-del-proyecto)
7. [Estructura del repositorio](#estructura-del-repositorio)
8. [Contribuciones](#contribuciones)
9. [Créditos](#créditos)
10. [Licencia](#licencia)

---

## **Descripción general**
- **Propósito:** *Ensamblar un robot de 17 DOF, desarrollar un firmware para poder controlarlo con una raspberry pi y lograr que se mueva, camine y responda a comandos.*  
- **Objetivos principales:**  
  - Crear nodos ROS para el control de las extremidades del robot.
  - Lograr el conexionado eléctrico y electrónico para su correcto funcionamiento.
  - Documentar los avances realizados para posibilitar futuras mejoras del sistema.

## **Características**
- Uso de **ROS 2 Jazzy Jalisco** en una Raspberry Pi 5 con Ubuntu 24.04 server.  
- Pruebas en hardware real.  
- Creación de una librería personalizada para asegurar la correcta comunicación entre la Raspberry y el Controlador de los Servos
- Independencia del nodo de control para asegurar su futura integración con sensores o controles.

## **Hardware:**  
- Raspberry Pi 5 con Ubuntu Server 24.04.  
- STM32, 32 Channel servo controller.
- Fuente de alimentación de entre 6 y 7 V con capacidad de picos de hasta 15A.
- Fuente de alimentación de 5V con capacidad de picos de 5A. 

## **Software:**  
- ROS 2 Jazzy Jalisco. 
- Python 3.10 o superior.
- Página web de control: Node.js
- Librerías adicionales: `rclpy`, `pyserial`, `RosBridge Suite`.  

## **Instalación**
Sigue estos pasos para configurar el proyecto en tu entorno:

1. **Clonar el repositorio:**
   ```bash
   git clone https://github.com/zebamanu/ProyectoT1T4
   cd ProyectoT1T4
   ```

2. **Instalar dependencias:**
   ```bash
   sudo apt update && sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
   sudo apt install nodejs
   sudo apt install npm
   sudo apt-get install ros-jazzy-rosbridge-suite
   pip install pyserial
   ```

3. **Configurar el entorno:**
   ```bash
   cd src
   source /install/local_setup.bash
   colcon build
   ```

4. **Configurar permisos de los dispositivos:**
   ```bash
   sudo chmod 666 /dev/ttyACM0  # Reemplazar con el puerto donde se conectó el controlador de servos
   ```

5. **Configurar la página Web**
   ```bash
   cd /Pagina_web
   npm install
   ```

## **Ejecución del proyecto**
Para ejecutar el sistema:  

1. **Lanzar el nodo del robot:**  
   ```bash
   ros2 run proyectoT1T4 nodo_robot
   ```

2. **Lanzar la página web:**
  ```bash
   cd /Pagina_web
   bash startRosbridge.sh & node index.js
  ```
3. **Acceder a la página web**
   La página es accesible desde cualquier dispositivo que esté conectado a la misma red de la raspberry a partir del url: http://[ip.asignada.a.la.raspberry]:3000

## **Estructura del repositorio**
```plaintext
├── Pagina_Web/                 # Código de la página web
│   ├── node_modules/           # Dependencias de nodejs
│   ├── index.html              # HTML de la interfaz web
│   ├── index.js                # Código Js de la página web con la conexión a RosBridge
│   ├── package.json            # Dependencias de nodeJs
│   └── startRosbridge.sh       # Bash que inicia el servicio del nodo RosBridge
├── src/                        # Código fuente del proyecto ROS
│   ├── build/                  # Código compilado
│   ├── install/                # Archivos de configuración
│   ├── log/                    # Registros de eventos
│   └── proyectoT1T4/proyectoT1T4        
│       ├── nodo_robot.py                   # Código fuente del nodo_robot
│       └── servoController.py              # Código fuente del controlador de servos por puerto serie
├── README.md                   # Este archivo
└── LICENCE                     # Licencia del Proyecto
```

## **Contribuciones**
- Abre un issue para reportar errores o sugerencias.  

## **Créditos**
- **Equipo:** [Matías Manuel Zeballos](https://github.com/zebamanu), [Matías Ezequiel Lugarzo](https://github.com/MatiLuga) & [Sabrina Luz Lombardo](https://github.com/SabriLomb)  
- **Supervisores:** Juan Luis Rosendo, Profesor adjunto de la cátedra de Introducción a la Robótica (E1501)
- **Institución:** [Facultad de Ingeniería, Universidad Nacional de La Plata](https://ing.unlp.edu.ar/)

## **Licencia**
Este proyecto está bajo la licencia [GNU 3.0](LICENSE.md).  
