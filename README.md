# Dobot-MG400-Ros2-Humble
***
Ejemplo de uso del Dobot MG400 en el semillero de robotica UAO para la clasificacion de objetos por color.

Para hacer uso de este repositorio solo es necesario crear una carpeta para contener la carpeta /src y realizar el siguiente comando:

$ colcon build

Posteriormente actualizar estos cambios:

$ source install/setup.bash 

Para usar el nodo de publicacion de imagenes a traves de una camara:

$ ros2 run dobot_mg400 img_publisher

Para usar el nodo de deteccion de colores:

$ ros2 run dobot_mg400 detector_colores 

Para usar el nodo de comandos al Dobot MG400:

$ ros2 run dobot_mg400 dobot 

El repositorio cuenta con un nodo de prueba para los comandos del Dobot MG400 el cual publica mensajes con los cuales se podran observar las diferentes trayectorias que realizara:

$ ros2 run dobot_mg400 minimal_publisher
