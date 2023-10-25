paquetes ROS probados:

LeGO-LOAM       f8e685b396cf19c5dcebf26cd8f39ee06a371e02 está para funcionar con <DATASET 1>
SC-LeGO-LOAM    fc1d0357e7958e069bc212da41ed2c6110cdac79 está para funcionar con <DATASET 1>

----------------------

DATASET 1:

zona mercadona, algunos de estos tiene el lidar inclinado.
imu con lo rojo para arriba

2023-06-14-16-28-42.bag ** este es el que usamos para las pruebas de carlos, los mapas del tfg vienen de este
2023-06-14-16-39-54.bag
2023-06-19-13-16-44.bag
2023-07-07-13-42-48.bag

ejemplo:
rosbag play 2023-06-14-16-28-42.bag --clock
roslaunch lego_loam run.launch

los PCD se guardan en /tmp
tfg@susana-hp:/tmp$ ll *.pcd
-rw-rw-r-- 1 tfg tfg  11M oct 25 13:31 cornerMap.pcd
-rw-rw-r-- 1 tfg tfg 105M oct 25 13:31 surfaceMap.pcd
-rw-rw-r-- 1 tfg tfg  27K oct 25 13:31 trajectory6DOF.pcd
-rw-rw-r-- 1 tfg tfg  11K oct 25 13:31 trajectory.pcd



si da error en 
>>pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.[pcl::VoxelGrid::applyFilter] Leaf size is too small >> for the input dataset. Integer indices would overflow

en: mapOptimization.cpp hay unas lineas. aumentar el leaf-size:
        downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for global map visualization

----------------------

DATASET 2:

zona espartales, 3+1 vueltas
3 loops, el ultimo mas grande
imu con lo rojo para abajo. lidar NO inclinado.
hay imagenes

invett_c4__2023-07-25-17-24-47.bag  *
invett_c4__2023-07-25-17-29-59.bag  *
invett_c4__2023-07-25-17-32-24.bag  *
invett_c4__2023-07-25-17-35-58.bag

con ninguno los bag con (*) pudimos cerrar el lazo bien con IMU, pero uno de estos se ha probado SIN imu y funciona.

dataset2 funciona solo SIN la imu. Con la imu al empezar se ve la nube de puntos al reves y respecto al eje para adelante la integracion "gira", como un tornillo.
para no incluir la imu, simplemente dar la lista de topics de esta forma.

rosbag play invett_c4__2023-07-25-17-24-47.bag --clock --topics /velodyne_points





los PCD se guardan en /tmp
tfg@susana-hp:/tmp$ ll *.pcd
-rw-rw-r-- 1 tfg tfg 3,4M oct 25 13:36 cornerMap.pcd
-rw-rw-r-- 1 tfg tfg  20M oct 25 13:36 finalCloud.pcd
-rw-rw-r-- 1 tfg tfg  26M oct 25 13:36 surfaceMap.pcd
-rw-rw-r-- 1 tfg tfg 4,7K oct 25 13:36 trajectory.pcd

-rw-rw-r-- 1 tfg tfg  27K oct 25 13:31 trajectory6DOF.pcd  *** ESTE no está (falta la parte del codigo en la version sc)


