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

## INGENIA-SE ROS package
[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

Here you can find ROS package for [INGENIA-SE project].
Follow next steps to install and run the simulation. Package include:
  - Enviroment
  - UAV controller
  - UGV controller
  - Server

# 1. Installation procedure
1) Git clone INGENIA-SE repo and move software file into your ROS_WS
2) Install dependencies:
    ```sh
    $ rosdep install --from-paths src -i
    ```
    In case you have another ROS_DISTRO, replace the word melodic with your DISTRO:
    ```sh
    $ rosdep install --from-paths src -i --rosdistro melodic --os=ubuntu:bionic
    ```
3) Build theworkspace
    ```sh
    $ catkin build
    ```
NOTES: Three pkgs will be ignored.
# 2. Running simulation
Despite it is a complex system based on multiple packages, simulation can be runned just with this command.
```sh
    $ roslaunch bebop_simulator simulation.launch
```
