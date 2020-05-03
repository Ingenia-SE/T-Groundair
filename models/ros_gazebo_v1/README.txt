Para probar esta versión del server, una vez hecho el make en el workspace y teniendo en un terminal roscore (01/05/2020):

Se requiere abrir dos nuevos terminales y acceder al workspace.
- En el primero de ellos, rosrun ros_gazebo_v1 interface [interfaz de usuario]
- En el otro, roslaunch ros_gazebo_v1 environment_gazebo.launch [simulación de gazebo]

De esta manera, se le enviará una orden a la caja que simula el dron para que envíe una imagen definida a la caja que actúa como server.
Aquí se procesará, reconocerán y ubicarán las bolas y colocará la primera de la lista la bola más cercana al (0,0).
Una vez concluido el proceso, se emitirán sendos mensajes de "Cycle ended".

NOTA - También se debe:
     - Dentro del workspace, colocar el archivo balls.h en la ruta devel/include/ros_gazebo_v1
     - Actualizar dentro de src/drone.cc la ruta de la imagen de prueba
