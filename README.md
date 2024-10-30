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

# Exemplo de Script de Simulação em Lua

Para editar o script de simulação, basta clicar duas vezes no ícone de script indicado na figura abaixo.  

<div align="center">

<img src="https://github.com/user-attachments/assets/aa6abaae-6e5b-496a-94c0-68b21ee85b04" alt="cena_exemplo" width="75%">

</div>

O exemplo da Figura é o [calmaN](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/scenes/CalmaN.ttt), o script associado é apresentado abaixo:


<div style="height: 400px; overflow-y: scroll; font-family: monospace; background-color: #f6f8fa; padding: 10px; border: 1px solid #ddd;">
<pre>
<code>
  1  sim=require'sim'
  2  simROS2=require'simROS2'
  3  
  4  function sysCall_init()
  5      robotHandle=sim.getObject('.')
  6      leftMotor=sim.getObject("./LeftMotor") -- Handle of the left motor
  7      rightMotor=sim.getObject("./RightMotor") -- Handle of the right motor
  8      lidarMotor=sim.getObject("./LidarMotor") -- Handle of the lidar motor
  9      Lidar = sim.getObject("./LidarMotor/LidarScan/Sensor") -- Handle of the lidar sensor
 10      drawingCont=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,200,{1,1,0},nil,nil,{1,1,0})
 11  
 12      -- Variaveis do Lidar 
 13      MotorLidarSpeed = 6*(2*math.pi) --360.0*5
 14      angle_min = 0.0
 15      angle_max =  2*math.pi
 16      angle_curr = 0.0
 17      scan_time = 0.0
 18      range_min = 0.12
 19      range_max = 2.0
 20      contador = 1
 21      ranges = {}
 22      local result, distance = sim.readProximitySensor(Lidar)
 23      ranges[1] = distance
 24      scan_data={}
 25      -- Launch the ROS2 client application:
 26      local VelTopicName='robot/cmd_vel'
 27      local EncoderTopicName='robot/encoder'
 28      local LidarTopicName='robot/lidar'
 29      local simulationTimeTopicName='robot/simTime'
 30      -- Prepare the sensor publisher and the motor speed subscribers:
 31      encoderPub=simROS2.createPublisher('/'..EncoderTopicName,'geometry_msgs/msg/Twist')
 32      lidarPub=simROS2.createPublisher('/'..LidarTopicName,'sensor_msgs/msg/LaserScan')
 33      simTimePub=simROS2.createPublisher('/'..simulationTimeTopicName,'std_msgs/msg/Float32')
 34      velSub=simROS2.createSubscription('/'..VelTopicName,'geometry_msgs/msg/Twist','setMotorVelocity_cb')
 35  end
 36  
 37  function setMotorVelocity_cb(msg)
 38      -- Left motor speed subscriber callback
 39      sim.setJointTargetVelocity(leftMotor,msg.angular.x)
 40      -- Right motor speed subscriber callback
 41      sim.setJointTargetVelocity(rightMotor,msg.angular.y)
 42  end
 43  
 44  
 45  function getTransformStamped(objHandle,name,relTo,relToName)
 46      t=sim.getSystemTime()
 47      p=sim.getObjectPosition(objHandle,relTo)
 48      o=sim.getObjectQuaternion(objHandle,relTo)
 49      return {
 50          header={
 51              stamp={sec=math.floor(t),nanosec=(t-math.floor(t))*10^9},
 52              frame_id=relToName
 53          },
 54          child_frame_id=name,
 55          transform={
 56              translation={x=p[1],y=p[2],z=p[3]},
 57              rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
 58          }
 59      }
 60  end
 61  
 62  function getAngleIncrement()
 63      local result,W
 64      W = sim.getObjectFloatParam(lidarMotor,sim.jointfloatparam_velocity)
 65      delta_time = sim.getSimulationTimeStep()
 66      delta_theta = W*delta_time
 67      return delta_theta, delta_time
 68  end
 69  
 70  function getLidarMeasurements()
 71      local delta_theta, delta_time, result, distance
 72      delta_theta, delta_time = getAngleIncrement()
 73      if (angle_curr + delta_theta) > angle_max then
 74         angle_curr = (angle_curr + delta_theta) - angle_max
 75         scan_time = 0.0
 76         contador = 1
 77         result, distance = sim.readProximitySensor(Lidar)
 78         ranges[contador] = distance
 79         simROS2.publish(lidarPub,scan_data) 
 80      else
 81          angle_curr = angle_curr + delta_theta
 82          scan_time = scan_time + delta_time
 83          contador = contador + 1
 84          result, distance = sim.readProximitySensor(Lidar)
 85          ranges[contador] = distance
 86      end
 87      scan_data['header'] = {
 88          stamp = simROS2.getTime(),
 89          frame_id = 'lidar_frame'}
 90      scan_data['angle_min']=angle_min 
 91      scan_data['angle_max']=angle_max 
 92      scan_data['angle_increment']=delta_theta 
 93      scan_data['time_increment']=delta_time 
 94      scan_data['scan_time']=scan_time 
 95      scan_data['range_min']=range_min 
 96      scan_data['range_max']=range_max 
 97      scan_data['ranges']=ranges
 98  end
 99  
100  function getEncoderMeasurements()
101      local wL,wR
102      wL = sim.getObjectFloatParam(leftMotor,sim.jointfloatparam_velocity)
103      wR = sim.getObjectFloatParam(rightMotor,sim.jointfloatparam_velocity)
104      local encoder_data = {linear = {x = 0, y = 0, z = 0}, angular = {x = 0, y = 0, z = 0}}
105      encoder_data.angular.x = wL
106      encoder_data.angular.y = wR
107      simROS2.publish(encoderPub,encoder_data)
108  end
109  
110  function sysCall_sensing() 
111      local p=sim.getObjectPosition(robotHandle)
112      sim.addDrawingObjectItem(drawingCont,p)
113      getLidarMeasurements()
114      getEncoderMeasurements()
115  end 
116  
117  function sysCall_actuation()
118      -- Define a velocidade do Motor do Lidar
119      sim.setJointTargetVelocity(lidarMotor,MotorLidarSpeed) --360*8
120      simROS2.publish(simTimePub,{data=sim.getSimulationTime()})
121      simROS2.sendTransform(getTransformStamped(robotHandle,'ros2InterfaceControlledBubbleRob',-1,'world'))
122  end
123  
124  function sysCall_cleanup()
125      simROS2.shutdownPublisher(encoderPub)
126      simROS2.shutdownPublisher(lidarPub)
127      simROS2.shutdownPublisher(simTimePub)
128      simROS2.shutdownSubscription(velSub)
129  end
</code>
</pre>
</div>


# Convertendo a Imagem 2D de um Mapa em um Objeto 3D

No loboratório de robótica da UFS, em alguns experimentos captamos as posições dos obstáculos usando câmera, resultando em um bitmap 2D do mapa. Dessa forma, para usar essa informação de forma simples e direta no CoppeliaSim, foi desenvolvido um código em MATLAB que converte a imagem do mapa em um arquivo `.obj`. Um exemplo é apresentado na figuras abaixo.

<div align="center">

|Imagem 2D| Conversão em 3D|
|------------------------------|------------------------------|
| ![Imagem 1](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/g1092.png) | ![Imagem 2](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/mapa01.png) |

</div>


Para gerar o arquivo `.obj`, é preciso representar os objetos como composição de paralelepípedos, fornecer os vértices de cada um desses paralelepípedos e indicar a as faces. Na figura abaixo pode ser visto um exemplo dos vértices obtidos da imagem 2D para conversão usando o código `image_to_obstacle_mesh.m`:

<div align="center">
<img src="https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/vertices.png" alt="vertices" width="50%">
</div>

O código `image_to_obstacle_mesh.m` implementa a função `image_to_obstacle(image_path, output_mesh_path, pixel_to_meter_ratio, obstacle_height)`. 
* `image_path` é o diretório da imagem
* `output_mesh_path` o diretório do arquivo `.obj`
* `pixel_to_meter_ratio` é a relação que define a escala entre pixel e metros, por exemplo `pixel_to_meter_ratio=0.1`, significa que o lado de um pixel equivale a 0,1 m na realidade.
* `obstacle_height` define a altura em unidades de pixel.






