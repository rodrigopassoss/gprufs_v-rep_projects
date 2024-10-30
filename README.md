# Modelos e Cenas do CoppeliaSim (V-REP)
Este repositório contém modelos de robôs e cenas do simulador CoppeliaSim (V-REP), com comunicação através do ROS 2. 

Os modelos:

<div align="center">

| Modelo       | Descrição       |
|----------------------|-------------------------|
| [calmaN](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/models/calmaN.obj)     | Modelo em 3d de uma versão protótipo do calmaN, desenvolvido no GPRUFS, com a simulação de um Lidar| 

</div>

As cenas de simulação:

<div align="center">

| Cena       | Descrição       |
|----------------------|-------------------------|
| [BubbleRobotDoNotCollide](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/scenes/BubbleRobotDoNotCollide.ttt)  | Cena para simulação do Bubble Robot, um simples robô de tração diferencial disponível no CoppeliaSim. Um controlador que pode ser usado para teste, chamado `walk` está disponível em [py_bubbleRobotController](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_bubbleRobotController). O comando para usar o controlador é: `ros2 run py_bubbleRobotController walk`.| 
| [calmaN](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/scenes/CalmaN.ttt)  | Cena para simulação do calmaN, robô desenvolvido no GPRUFS. Pacotes para utilizar junto com essa simulação estão disponíveis em [py_calmaN](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_calmaN). | 
| [Teste com Câmera](ros2InterfaceTopicPublisherAndSubscriber-lua.ttt)  | Cena para simulação da Câmera. Essa cena foi preparada para ser usada junto com o pacote [py_camera](https://github.com/rodrigopassoss/gprufs_ros2_packages/tree/main/py_camera), utilizando o comando `ros2 run py_camera camera_sub`| 

</div>

* Obs.: Cada cena contém scripts em lua (uma linguagem de programação brasileira!), que configura os nós e os tópicos do ROS 2 que vão se comunicar com os códigos em python ([gprufs_ros2_packages](https://github.com/rodrigopassoss/gprufs_ros2_packages)) e Matlab/Octave ([gprufs_ros2_udp](https://github.com/rodrigopassoss/gprufs_ros2_udp)). Um tutorial sobre como usar o ROS 2 no CoppeliaSim e como configurar o script para interface com os tópicos é disponibilizado pela própria equipe do Coppelia e pode ser acessado [aqui](https://manual.coppeliarobotics.com/) (Tutorials->ROS Tutorial->ROS 2 Tutorial).

Exemplo de Cena (BubbleRobotDoNotCollide):

<div align="center">

<img src="https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/cena_exemplo.png" alt="cena_exemplo" width="50%">

</div>

Note que as cenas possuem dois tipos de objetos principais: Obstáculos e Robôs. Ambos são modelos 3d que podem ser gerados a partir de arquivos `.obj`, isto é, em posso de modelos 3d dos robôs é possível importar esses robôs para a plataforma. Da mesma forma, com modelos 3d dos obstáculos é possível adicioná-los a cena.

# Convertendo a Imagem 2D de um Mapa em um Objeto 3D

No loboratório de robótica da UFS, em alguns experimentos captamos as posições dos obstáculos usando câmera, resultando em um bitmap 2D do mapa. Dessa forma, para usar essa informação de forma simples e direta no CoppeliaSim, foi desenvolvido um código em MATLAB que converte a imagem do mapa em um arquivo `.obj`. Um exemplo é apresentado na figura abaixo.

|Imagem 2D| Conversão em 3D|
|------------------------------|------------------------------|
| ![Imagem 1](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/g1092.png) | ![Imagem 2](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/mapa01.png) |







