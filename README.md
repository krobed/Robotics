# Pasos para correr la simulación:  

  
1. Instalar ROS2 jazzy y requerimientos.
2. Correr RViz y Gazebo:
   ``` shell
    ros2 launch el7009_diff_drive_robot robot.launch.py
   ```
3. Cargar mapa:
   ``` shell
    ros2 launch el7009_diff_drive_robot localization.launch.py
   ```
4. Enviar poses predefinidas:
   ``` shell
    ros2 run el7009_diff_drive_robot send_goal.py
   ``` 
 En caso de querer guardar trayectorias:
   ``` shell
    ros2 run el7009_diff_drive_robot plotpose.py
   ```


En caso de no querer usar un plan global generado por el algoritmo RRT*, es posible omitir el paso 4 y enviar un plan de forma manual. El código para enviar un plan se encuentra en `path_msg.txt`, mientras que el código para enviar una meta se encuentra en el archivo `goal_pose_msg.txt`.
