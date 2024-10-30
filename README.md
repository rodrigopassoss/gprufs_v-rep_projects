# Modelos e Cenas do CoppeliaSim (V-REP)
Este repositório contém modelos de robôs e cenas do simulador CoppeliaSim (V-REP), com comunicação através do ROS 2. 

Os modelos:

<div align="center">

| Modelo       | Descrição       |
|----------------------|-------------------------|
| [calmaN](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/models/calmaN.obj)     | Modelo em 3d de uma versão protótipo do calmaN, desenvolvido no GPRUFS, com a simulação de um Lidar| 

</div>

As cenas de simulação:

| Cena       | Descrição       |
|----------------------|-------------------------|
| [BubbleRobotDoNotCollide](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/scenes/BubbleRobotDoNotCollide.ttt)  | Cena para simulação do Bubble Robot, um simples robô de tração diferencial disponível no CoppeliaSim. Um controlador que pode ser usado para teste, chamado `walk` está disponível em [py_bubbleRobotController](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_bubbleRobotController). O comando para usar o controlador é: `ros2 run py_bubbleRobotController walk`.| 
| [calmaN](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/scenes/CalmaN.ttt)  | Cena para simulação do calmaN, robô desenvolvido no GPRUFS. Pacotes para utilizar junto com essa simulação estão disponíveis em [py_calmaN](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_calmaN). | 
| [Teste com Câmera](ros2InterfaceTopicPublisherAndSubscriber-lua.ttt)  | Cena para simulação da Câmera. Essa cena foi preparada para ser usada junto com o pacote [py_camera](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_camera), utilizando o comando `ros2 run py_camera camera_sub`| 

Exemplo de Cena:

[cena_exemplo](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/cena_exemplo.png)



