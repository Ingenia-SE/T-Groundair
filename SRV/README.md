Para probar esta versión del server, una vez hecho el make/build en el workspace se ejecutará la sentencia (11/05/2020):

>> roslaunch ros_gazebo_v1 environment_gazebo.launch

En la interfaz se deberá introducir un "1" para iniciar el sistema.
Con ello, se dará comienzo a una serie de "movimientos" del UAV hasta llegar al punto deseado para tomar una imagen.
En este momento, la caja que simula el dron enviará una imagen definida a la caja que actúa como server.
Aquí se procesará, reconocerán y ordenarán las bolas para conseguir la ruta óptima del UGV, suponiendo en este caso que se encuentre en el (0,0).
Después, el UAV volverá a su posición inicial y, entre tanto, el UGV "comenzará" (virtualmente) a capturar las bolas.
Una vez "concluido el proceso", se emitirán un mensaje de "Cycle ended".

NOTA - También se debe actualizar dentro de src/drone.cc la ruta de la imagen de prueba
