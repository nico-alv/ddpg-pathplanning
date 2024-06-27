load Environment1 BW

mapMatrix = BW;
mapScale = 1;

scanAngles = -3*pi/8:pi/12:3*pi/8; % Ángulos lidar
maxRange = 70; % Alcance máximo
lidarNoiseVariance = 0.1^2;
lidarNoiseSeeds = randi(intmax, size(scanAngles));

% Parámetros de velocidad máxima
maxLinSpeed = 0.9; % Velocidad lineal
maxAngSpeed = 0.9; % Velocidad angular

%%%%route1environment1%%%%%
initX = 34; % Punto de inicio x
initY = 31; % Punto de inicio y
%%%%%%%%%%%%%%
initTheta = pi/2; % Ángulo inicial

fig = figure("Name", "BW");
set(fig, "Visible", "on");
ax = axes(fig);

show(binaryOccupancyMap(mapMatrix), "Parent", ax);
hold on
plotTransforms([initX, initY, 0], eul2quat([initTheta, 0, 0]), "MeshFilePath", "groundvehicle.stl", "View", "2D");
light;
hold off

%% Modelo de Simulink y configuración del agente
mdl = "exampleHelperAvoidObstaclesMobileRobot";
Tfinal = 1000;
sampleTime = 0.1;

agentBlk = mdl + "/Agent";
open_system(mdl)
open_system(mdl + "/Environment")

obsInfo = rlNumericSpec([numel(scanAngles) 1], ...
    "LowerLimit", zeros(numel(scanAngles), 1), ...
    "UpperLimit", ones(numel(scanAngles), 1) * maxRange);
numObservations = obsInfo.Dimension(1);

numActions = 2;
actInfo = rlNumericSpec([numActions 1], ...
    "LowerLimit", -1, ...
    "UpperLimit", 1);

env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @(in) exampleHelperRLAvoidObstaclesResetFcn(in, scanAngles, maxRange, mapMatrix);
env.UseFastRestart = "On";

% Redes del Crítico y del Actor
statePath = [
    featureInputLayer(numObservations, "Normalization", "none", "Name", "State")
    fullyConnectedLayer(50, "Name", "CriticStateFC1")
    reluLayer("Name", "CriticRelu1")
    fullyConnectedLayer(25, "Name", "CriticStateFC2")];
actionPath = [
    featureInputLayer(numActions, "Normalization", "none", "Name", "Action")
    fullyConnectedLayer(25, "Name", "CriticActionFC1")];
commonPath = [
    additionLayer(2, "Name", "add")
    reluLayer("Name", "CriticCommonRelu")
    fullyConnectedLayer(1, "Name", "CriticOutput")];

% Creación de dos redes críticas
criticNetwork1 = layerGraph();
criticNetwork1 = addLayers(criticNetwork1, statePath);
criticNetwork1 = addLayers(criticNetwork1, actionPath);
criticNetwork1 = addLayers(criticNetwork1, commonPath);
criticNetwork1 = connectLayers(criticNetwork1, "CriticStateFC2", "add/in1");
criticNetwork1 = connectLayers(criticNetwork1, "CriticActionFC1", "add/in2");
criticNetwork1 = dlnetwork(criticNetwork1);

criticNetwork2 = layerGraph();
criticNetwork2 = addLayers(criticNetwork2, statePath);
criticNetwork2 = addLayers(criticNetwork2, actionPath);
criticNetwork2 = addLayers(criticNetwork2, commonPath);
criticNetwork2 = connectLayers(criticNetwork2, "CriticStateFC2", "add/in1");
criticNetwork2 = connectLayers(criticNetwork2, "CriticActionFC1", "add/in2");
criticNetwork2 = dlnetwork(criticNetwork2);

% Configuración del Agente TD3
agentOpts = rlTD3AgentOptions(...
    "SampleTime", sampleTime, ...
    "ActorOptimizerOptions", actorOptions, ...
    "CriticOptimizerOptions", criticOptions, ...
    "DiscountFactor", 0.995, ...
    "MiniBatchSize", 128, ...
    "ExperienceBufferLength", 1e6, ...
    "TargetSmoothFactor", 0.005, ...
    "TargetUpdateFrequency", 2, ...
    "ExplorationModel", "OrnsteinUhlenbeck", ...
    "Variance", 0.1, ...
    "VarianceDecayRate", 1e-5);

% Creación del Agente TD3
obstacleAvoidanceAgent = rlTD3Agent(actor, [critic1, critic2], agentOpts);
open_system(mdl + "/Agent")

%% Entrenamiento y simulación
maxEpisodes = 1000;
maxSteps = ceil(Tfinal / sampleTime);
trainOpts = rlTrainingOptions(...
    "MaxEpisodes", maxEpisodes, ...
    "MaxStepsPerEpisode", maxSteps, ...
    "ScoreAveragingWindowLength", 50, ...
    "StopTrainingCriteria", "AverageReward", ...
    "StopTrainingValue", 9999, ...
    "Verbose", true, ...
    "Plots", "training-progress");

doTraining = true; % Cambia esto a true para entrenar

if doTraining
    % Entrenar al agente
    trainingStats = train(obstacleAvoidanceAgent, env, trainOpts);
else
    % Cargar un agente preentrenado
    load exampleHelperAvoidObstaclesAgent obstacleAvoidanceAgent
end

set_param("exampleHelperAvoidObstaclesMobileRobot", "StopTime", "850");
out = sim("exampleHelperAvoidObstaclesMobileRobot.slx");

for i = 1:5:size(out.range, 3)
   u = out.pose(i, :);
   r = out.range(:, :, i);
   exampleHelperAvoidObstaclesPosePlot(u, mapMatrix, mapScale, r, scanAngles, ax);
end

% Crear una figura
figure;

% Invertir los valores del mapa binario
BW_inverted = imcomplement(BW);

% Mostrar el mapa binario invertido como una imagen de fondo
imshow(BW_inverted);
hold on; % Mantener el gráfico actual para superponer la trayectoria

% Definir los límites del gráfico usando las dimensiones de tu mapa 2D
xLimits = [1 size(BW, 2)];
yLimits = [1 size(BW, 1)];
xlim(xLimits);
ylim(yLimits);

% Crear una matriz vacía para almacenar la trayectoria del robot
trajectory = [];

% Iterar a través de los datos de la trayectoria y trazar la trayectoria del robot
for i = 1:5:size(out.range, 3)
    u = out.pose(i, :);
    % Ajustar las coordenadas del eje Y
    u(2) = (-1) * (u(2) - 256);
    trajectory = [trajectory; u];
end

% Trazar la trayectoria en el mapa 2D
plot(trajectory(:, 1), trajectory(:, 2), 'r-', 'LineWidth', 2);

% Etiquetas y título
xlabel('Eje X');
ylabel('Eje Y');
title('Mapa Binario con Trayectoria del Robot');

% Marcar el punto de inicio con color verde
startPoint = trajectory(1, :);
plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Marcar el punto de término con color rojo
endPoint = trajectory(end, :);
plot(endPoint(1), endPoint(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Mostrar la leyenda
legend('Trayectoria', 'Inicio', 'Término');

% Habilitar la cuadrícula
grid on;

% Desactivar el modo de espera para permitir que la figura se muestre correctamente
drawnow;
