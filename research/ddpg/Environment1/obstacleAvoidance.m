load Environment1 BW

mapMatrix = BW;
mapScale = 1;

scanAngles = -3*pi/8:pi/12:3*pi/8; % Ángulos del LIDAR
maxRange = 70; % Rango máximo
lidarNoiseVariance = 0.1^2;
lidarNoiseSeeds = randi(intmax, size(scanAngles));

% Parámetros de velocidad máxima
maxLinSpeed = 0.9; % Velocidad lineal
maxAngSpeed = 0.9; % Velocidad angular

% Punto de inicio
initX = 34;
initY = 31;
initTheta = pi/2; % Ángulo inicial

% Visualización del mapa
fig = figure("Name", "BW");
set(fig, "Visible", "on");
ax = axes(fig);
show(binaryOccupancyMap(mapMatrix), "Parent", ax);
hold on
plotTransforms([initX, initY, 0], eul2quat([initTheta, 0, 0]), "MeshFilePath", "groundvehicle.stl", "View", "2D");
light;
hold off

% Configuración del modelo de Simulink
mdl = "exampleHelperAvoidObstaclesMobileRobot";
Tfinal = 1000;
sampleTime = 0.1;
agentBlk = mdl + "/Agent";
open_system(mdl)
open_system(mdl + "/Environment")

% Especificaciones de observación y acción
obsInfo = rlNumericSpec([numel(scanAngles) 1], "LowerLimit", zeros(numel(scanAngles), 1), "UpperLimit", ones(numel(scanAngles), 1) * maxRange);
actInfo = rlNumericSpec([2 1], "LowerLimit", [-1 -1], "UpperLimit", [1 1]);

% Crear entorno de Simulink
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @(in)exampleHelperRLAvoidObstaclesResetFcn(in, scanAngles, maxRange, mapMatrix);
env.UseFastRestart = "On";

% Configuración de las redes para TD3
criticNetwork1 = createCriticNetwork(obsInfo.Dimension(1), actInfo.Dimension(1), "Critic1");
criticNetwork2 = createCriticNetwork(obsInfo.Dimension(1), actInfo.Dimension(1), "Critic2");
actorNetwork = createActorNetwork(obsInfo.Dimension(1), actInfo.Dimension(1));

% Crear opciones del agente TD3
agentOpts = rlTD3AgentOptions(...
    "SampleTime", sampleTime, ...
    "ActorOptimizerOptions", rlOptimizerOptions("LearnRate", 1e-4, "GradientThreshold", 1), ...
    "CriticOptimizerOptions", rlOptimizerOptions("LearnRate", 1e-3, "GradientThreshold", 1), ...
    "DiscountFactor", 0.99, ...
    "MiniBatchSize", 128, ...
    "ExperienceBufferLength", 1e6, ...
    "TargetSmoothFactor", 0.005, ...
    "TargetUpdateFrequency", 4);

% Crear el agente TD3
agent = rlTD3Agent(actorNetwork, [criticNetwork1, criticNetwork2], agentOpts);

% Configuración de entrenamiento
maxEpisodes = 1000;
maxSteps = ceil(Tfinal/sampleTime);
trainOpts = rlTrainingOptions(...
    "MaxEpisodes", maxEpisodes, ...
    "MaxStepsPerEpisode", maxSteps, ...
    "ScoreAveragingWindowLength", 50, ...
    "StopTrainingCriteria", "AverageReward", ...
    "StopTrainingValue", 9999, ...
    "Verbose", true, ...
    "Plots", "training-progress");

% Iniciar entrenamiento
doTraining = true;
if doTraining
    trainingStats = train(agent, env, trainOpts);
else
    load exampleHelperAvoidObstaclesAgent obstacleAvoidanceAgent
end

% Funciones adicionales para crear las redes
function criticNet = createCriticNetwork(numObservations, numActions, name)
    statePath = [
        featureInputLayer(numObservations, "Normalization", "none", "Name", "state")
        fullyConnectedLayer(400, "Name", "CriticStateFC1")
        reluLayer("Name", "CriticRelu1")
        fullyConnectedLayer(300, "Name", "CriticStateFC2")
        additionLayer(2, "Name", "add")];
    actionPath = [
        featureInputLayer(numActions, "Normalization", "none", "Name", "action")
        fullyConnectedLayer(300, "Name", "CriticActionFC1")];
    commonPath = [
        reluLayer("Name", "CriticCommonRelu")
        fullyConnectedLayer(1, "Name", "CriticOutput", "BiasLearnRateFactor", 0)];
    criticNet = layerGraph(statePath);
    criticNet = addLayers(criticNet, actionPath);
    criticNet = addLayers(criticNet, commonPath);
    criticNet = connectLayers(criticNet, "CriticStateFC2", "add/in1");
    criticNet = connectLayers(criticNet, "CriticActionFC1", "add/in2");
    criticNet = dlnetwork(criticNet);
end

function actorNet = createActorNetwork(numObservations, numActions)
    layers = [
        featureInputLayer(numObservations, "Normalization", "none", "Name", "state")
        fullyConnectedLayer(400, "Name", "actorFC1")
        reluLayer("Name", "actorRelu1")
        fullyConnectedLayer(300, "Name", "actorFC2")
        reluLayer("Name", "actorRelu2")
        fullyConnectedLayer(numActions, "Name", "actorFC3")
        tanhLayer("Name", "ActionTanh")
        scalingLayer("Name", "ActorScaling", "Scale", [1, 1])];  % Ajustar según los límites de las acciones
    actorNet = dlnetwork(layers);
end
