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


## Exemplo de Cena (BubbleRobotDoNotCollide):

<div align="center">

<img src="https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/cena_exemplo.png" alt="cena_exemplo" width="50%">

</div>

Note que as cenas possuem dois tipos de objetos principais: Obstáculos e Robôs. Ambos são modelos 3d que podem ser gerados a partir de arquivos `.obj`, isto é, em posso de modelos 3d dos robôs é possível importar esses robôs para a plataforma. Da mesma forma, com modelos 3d dos obstáculos é possível adicioná-los a cena.

* Obs.: Cada cena contém scripts em lua (uma linguagem de programação brasileira!), que configura os nós e os tópicos do ROS 2 que vão se comunicar com os códigos em python ([gprufs_ros2_packages](https://github.com/rodrigopassoss/gprufs_ros2_packages)) e Matlab/Octave ([gprufs_ros2_udp](https://github.com/rodrigopassoss/gprufs_ros2_udp)). Um tutorial sobre como usar o ROS 2 no CoppeliaSim e como configurar o script para interface com os tópicos é disponibilizado pela própria equipe do Coppelia e pode ser acessado [aqui](https://manual.coppeliarobotics.com/) (Tutorials->ROS Tutorial->ROS 2 Tutorial).

## Exemplo de Script de Simulação em Lua

Para editar o script de simulação, basta clicar duas vezes no ícone de script indicado na figura abaixo.  

<div align="center">

<img src="https://github.com/user-attachments/assets/aa6abaae-6e5b-496a-94c0-68b21ee85b04" alt="cena_exemplo" width="75%">

</div>

O exemplo da Figura é o [calmaN](https://github.com/rodrigopassoss/gprufs_v-rep_projects/blob/main/scenes/CalmaN.ttt), o script associado é apresentado abaixo:


<div style="height: 400px; overflow-y: scroll; font-family: monospace; background-color: #f6f8fa; padding: 10px; border: 1px solid #ddd;">
<pre>
<code>
--lua

sim=require'sim'
simROS2=require'simROS2'

function sysCall_init()
    robotHandle=sim.getObject('.')
    leftMotor=sim.getObject("./LeftMotor") -- Handle of the left motor
    rightMotor=sim.getObject("./RightMotor") -- Handle of the right motor
    lidarMotor=sim.getObject("./LidarMotor") -- Handle of the lidar motor
    Lidar = sim.getObject("./LidarMotor/LidarScan/Sensor") -- Handle of the lidar sensor
    drawingCont=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,200,{1,1,0},nil,nil,{1,1,0})
    -- Variaveis do Lidar 
    MotorLidarSpeed = 6*(2*math.pi) --360.0*5
    angle_min = 0.0
    angle_max =  2*math.pi
    angle_curr = 0.0
    scan_time = 0.0
    range_min = 0.12
    range_max = 2.0
    contador = 1
    ranges = {}
    local result, distance = sim.readProximitySensor(Lidar)
    ranges[1] = distance
    scan_data={}
    -- Launch the ROS2 client application:
    local VelTopicName='robot/cmd_vel'
    local EncoderTopicName='robot/encoder'
    local LidarTopicName='robot/lidar'
    local simulationTimeTopicName='robot/simTime'
    -- Prepare the sensor publisher and the motor speed subscribers:
    encoderPub=simROS2.createPublisher('/'..EncoderTopicName,'geometry_msgs/msg/Twist')
    lidarPub=simROS2.createPublisher('/'..LidarTopicName,'sensor_msgs/msg/LaserScan')
    simTimePub=simROS2.createPublisher('/'..simulationTimeTopicName,'std_msgs/msg/Float32')
    velSub=simROS2.createSubscription('/'..VelTopicName,'geometry_msgs/msg/Twist','setMotorVelocity_cb')
end

function setMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(leftMotor,msg.angular.x)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(rightMotor,msg.angular.y)
end

function getTransformStamped(objHandle,name,relTo,relToName)
    t=sim.getSystemTime()
    p=sim.getObjectPosition(objHandle,relTo)
    o=sim.getObjectQuaternion(objHandle,relTo)
    return {
        header={
            stamp={sec=math.floor(t),nanosec=(t-math.floor(t))*10^9},
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

function getAngleIncrement()
    local result,W
    W = sim.getObjectFloatParam(lidarMotor,sim.jointfloatparam_velocity)
    delta_time = sim.getSimulationTimeStep()
    delta_theta = W*delta_time
    return delta_theta, delta_time
end

function getLidarMeasurements()
    local delta_theta, delta_time, result, distance
    delta_theta, delta_time = getAngleIncrement()
    if (angle_curr + delta_theta) > angle_max then
       angle_curr = (angle_curr + delta_theta) - angle_max
       scan_time = 0.0
       contador = 1
       result, distance = sim.readProximitySensor(Lidar)
       --if distance<=0.0 or distance>2.0 then
       --     ranges[contador] = 2.0
       --else
            ranges[contador] = distance
       --end
       simROS2.publish(lidarPub,scan_data) 
    else
        angle_curr = angle_curr + delta_theta
        scan_time = scan_time + delta_time
        contador = contador + 1
        result, distance = sim.readProximitySensor(Lidar)
        --if distance<=0.0 or distance>2.0 then
        --    ranges[contador] = 2.0
        --else
            ranges[contador] = distance
        --end 
    end
    -- 
    --local scan_data={}
    scan_data['header'] = {
        stamp = simROS2.getTime(),
        frame_id = 'lidar_frame'}
    scan_data['angle_min']=angle_min 
    scan_data['angle_max']=angle_max 
    scan_data['angle_increment']=delta_theta 
    scan_data['time_increment']=delta_time 
    scan_data['scan_time']=scan_time 
    scan_data['range_min']=range_min 
    scan_data['range_max']=range_max 
    scan_data['ranges']=ranges
    --simROS2.publish(lidarPub,scan_data) 
end

function getEncoderMeasurements()
    local wL,wR
    wL = sim.getObjectFloatParam(leftMotor,sim.jointfloatparam_velocity)
    wR = sim.getObjectFloatParam(rightMotor,sim.jointfloatparam_velocity)
    local encoder_data = {linear = {x = 0, y = 0, z = 0}, angular = {x = 0, y = 0, z = 0}}
    encoder_data.angular.x = wL
    encoder_data.angular.y = wR
    simROS2.publish(encoderPub,encoder_data)
end

function sysCall_sensing() 
    local p=sim.getObjectPosition(robotHandle)
    sim.addDrawingObjectItem(drawingCont,p)
    getLidarMeasurements()
    getEncoderMeasurements()
end 

function sysCall_actuation()
    -- Define a velocidade do Motor do Lidar
    sim.setJointTargetVelocity(lidarMotor,MotorLidarSpeed) --360*8
    simROS2.publish(simTimePub,{data=sim.getSimulationTime()})
    -- Send the robot's transform:
    simROS2.sendTransform(getTransformStamped(robotHandle,'ros2InterfaceControlledBubbleRob',-1,'world'))
    -- To send several transforms at once, use simROS2.sendTransforms instead
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    simROS2.shutdownPublisher(encoderPub)
    simROS2.shutdownPublisher(lidarPub)
    simROS2.shutdownPublisher(simTimePub)
    simROS2.shutdownSubscription(velSub)
end

</code>
</pre>
</div>

As funções `sysCall_init()` e `sysCall_cleanup()` são executadas apenas uma vez, na inicialização e na finalização, respectivamente. As funções `sysCall_sensing()` e `sysCall_actuation()` são executadas a cada instante de amostragem da simulação, e são usadas para simular os sensores e atuadores, respectivamente. As demais funções são funções auxiliares. A interface com os tópicos do ROS 2 é configurada na função `sysCall_init()` no seguinte trecho:

<pre>
-- Prepare the sensor publisher and the motor speed subscribers:
encoderPub=simROS2.createPublisher('/'..EncoderTopicName,'geometry_msgs/msg/Twist')
lidarPub=simROS2.createPublisher('/'..LidarTopicName,'sensor_msgs/msg/LaserScan')
simTimePub=simROS2.createPublisher('/'..simulationTimeTopicName,'std_msgs/msg/Float32')
velSub=simROS2.createSubscription('/'..VelTopicName,'geometry_msgs/msg/Twist','setMotorVelocity_cb')
</pre>

As informações dos sensores são publicadas nos tópicos do ROS 2 sempre que a função `sysCall_sensing()` é chamada, por meio das funções `getLidarMeasurements()` e `getEncoderMeasurements()`. Já os dados de velocidades são atualizadas via *callback*, com a função `setMotorVelocity_cb(msg)`.

## Convertendo a Imagem 2D de um Mapa em um Objeto 3D

No loboratório de robótica da UFS, em alguns experimentos, captamos as posições dos obstáculos usando câmera, resultando em um bitmap 2D do mapa. Dessa forma, para usar essa informação de forma simples e direta no CoppeliaSim, foi desenvolvido um código em MATLAB que converte a imagem do mapa em um arquivo `.obj`. Um exemplo é apresentado na figuras abaixo.

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






