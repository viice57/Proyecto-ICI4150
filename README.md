# Proyecto ICI4150

El repositorio contiene toda la documentación asociada a planificación, diseño, desarrollo y simulación de un robot móvil, como parte de las exigencias para el proyecto semestral del ramo *Robótica y Sistemas Autónomos*.

* **Objetivo:** Desarrolllar un robot móvil autónomo en Webots, capaz de navegar por un entorno simulado evitando obstáculos, generando un mapa básico y planificando
rutas para alcanzar objetivos definidos.

* **Fecha de entrega:** 26 de junio de 2025.
 
## Integrantes

- Cristofer Contreras
- Ignacio Cuevas
- Vicente Morales

## Contenidos

Dentro del repositorio, está incluido lo siguiente:

- **Video** sobre del funcionamiento: Se encuentra en la carpeta **Otros**
- **Informe final:** Se encuentra en la carpeta **Otros**
- **Código fuente:** Se encuentra en la carpeta **Webots**

## Recursos

- [Enlace al repositorio](https://github.com/viice57/Proyecto-ICI4150.git)
- [Enlace al informe](https://github.com/viice57/Proyecto-ICI4150/Otros/InformeFinal.pdf)
- [Enlace al video](https://github.com/viice57/Proyecto-ICI4150/Otros/Prueba.mp4)
- [Documentación de Webots](https://cyberbotics.com/doc)
- [Comunidad de Webots](https://forum.cyberbotics.com/)

## Instrucciones de ejecución

El siguiente conjunto de instrucciones describe los pasos para ejecutar el proyecto en el simulador Webots.

### Requisitos Previos

1. **Instalar Webots**
   - Si aún no tienes Webots instalado, descárgalo desde el [sitio oficial](https://cyberbotics.com/). Disponible para Windows, macOS y Linux.

2. **Configurar el entorno de desarrollo**
   - Debes de tener instalado un compilador de C/C++.

3. **Instalar librerías si no las tienes**
   - Instala las librerías necesarias usando el gestor de paquetes adecuado según tu sistema operativo.
     ```bash
     sudo apt-get install libwebots-dev
     ```

### Ejecución del Proyecto

1. **Abrir el Proyecto**
   - Navega a la carpeta donde se encuentra el proyecto.
   - Busca el archivo con extensión `.wbt` (archivo de mundo), puedes abrirlo directamente desde la interfaz de Webots:
     - Abre Webots.
     - Haz clic en `File > Open World` y selecciona el archivo de mundo (`.wbt`).

2. **Configurar el Mundo y Controladores**
   - Asegúrate de que el archivo de configuración del mundo (`world file`) esté correctamente definido y que los controladores del robot estén funcionando.

3. **Ejecutar la Simulación**
   - Haz clic en el botón `Play` (el triángulo gris en la barra de herramientas) en la interfaz de Webots para iniciar la simulación.
   - La simulación debería comenzar y se podrá ver la interacción de del robot en el entorno.

4. **Monitorear la Simulación**
   - Durante la ejecución, es posible ver la información del robot, las métricas de simulación y otras herramientas de diagnóstico en la interfaz de Webots.

5. **Finalizar la Simulación**
   - Para detener la simulación, haz clic en el botón `Stop` (el cuadrado gris en la barra de herramientas).
   - Puedes reiniciar la simulación o hacer ajustes según lo estimes conveniente.

6. **Guardar mundos**
    - Puedes guardar la simulación para retomarla más tarde desde `File > Save World`. O también directamente desde el diskette gris de la barra de herramientas.
