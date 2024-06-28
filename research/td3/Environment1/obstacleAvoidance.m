% Cargar el entorno desde un archivo guardado
load Environment1 BW

% Definir la matriz del mapa y escalar
mapMatrix = BW;
mapScale = 1;

% Configuración del LIDAR
scanAngles = linspace(-pi/4, pi/4, 10);  % 10 ángulos de escaneo
maxRange = 10;  % Rango máximo en metros
lidarNoise = 0.1;  % Ruido del sensor

% Parámetros de velocidad del robot
maxLinSpeed = 0.5;  % Velocidad lineal máxima (m/s)
maxAngSpeed = pi/4;  % Velocidad angular máxima (rad/s)

% Punto de inicio del robot
initX = 10;  % Posición inicial X
initY = 10;  % Posición inicial Y
initTheta = 0;  % Orientación inicial

% Configurar el entorno de Simulink
mdl = 'MobileRobotSimulation';  % Nombre del modelo de Simulink
open_system(mdl);

% Configurar el bloque de agente en Simulink
agentBlk = [mdl, '/RL Agent'];

% Especificaciones del entorno
obsInfo = rlNumericSpec([numel(scanAngles) 1], 'LowerLimit', 0, 'UpperLimit', maxRange);
actInfo = rlNumericSpec([2 1], 'LowerLimit', [-maxLinSpeed; -maxAngSpeed], 'UpperLimit', [maxLinSpeed; maxAngSpeed]);

% Crear el entorno de Simulink
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @(in)localResetFcn(in);

% Definir las capas de la red para el crítico
criticLayerSizes = [400 300];
statePath = [
    featureInputLayer(numel(scanAngles), 'Normalization', 'none', 'Name', 'observation')
    fullyConnectedLayer(criticLayerSizes(1), 'Name', 'CriticStateFC1')
    reluLayer('Name', 'CriticRelu1')
    fullyConnectedLayer(criticLayerSizes(2), 'Name', 'CriticStateFC2')
    additionLayer(2, 'Name', 'add')
    reluLayer('Name', 'CriticCommonRelu')
    fullyConnectedLayer(1, 'Name', 'CriticOutput')];

actionPath = [
    featureInputLayer(2, 'Normalization', 'none', 'Name', 'action')
    fullyConnectedLayer(criticLayerSizes(2), 'Name', 'CriticActionFC1')];

% Conectar la entrada de acción a la segunda capa totalmente conectada
criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = connectLayers(criticNetwork, 'CriticActionFC1', 'add/in2');

% Crear y configurar los agentes TD3
options = rlTD3AgentOptions(...
    'SampleTime', 0.1, ...
    'TargetSmoothFactor', 1e-3, ...
    'ExperienceBufferLength', 1e6, ...
    'MiniBatchSize', 64, ...
    'DiscountFactor', 0.99, ...
    'TargetUpdateFrequency', 4);

% Crear el actor y el crítico
actorNetwork = [
    featureInputLayer(numel(scanAngles), 'Normalization', 'none', 'Name', 'observation')
    fullyConnectedLayer(300, 'Name', 'actorFC1')
    reluLayer('Name', 'actorRelu1')
    fullyConnectedLayer(200, 'Name', 'actorFC2')
    reluLayer('Name', 'actorRelu2')
    fullyConnectedLayer(2, 'Name', 'actorFC3')
    tanhLayer('Name', 'ActionTanh')
    scalingLayer('Name', 'ActorScaling', 'Scale', [maxLinSpeed; maxAngSpeed])];
actorNetwork = dlnetwork(actorNetwork);

critic1 = rlQValueFunction(criticNetwork, obsInfo, actInfo, 'Observation', {'observation'}, 'Action', {'action'});
critic2 = rlQValueFunction(criticNetwork, obsInfo, actInfo, 'Observation', {'observation'}, 'Action', {'action'});

actor = rlContinuousDeterministicActor(actorNetwork, obsInfo, actInfo);
agent = rlTD3Agent(actor, [critic1, critic2], options);

% Configurar y correr el entrenamiento
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 500, ...
    'MaxStepsPerEpisode', 200, ...
    'Verbose', true, ...
    'Plots', 'training-progress', ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 200);

trainingStats = train(agent, env, trainOpts);
