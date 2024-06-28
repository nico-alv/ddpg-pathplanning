% Cargar el entorno
load Modified_Environment1 BW
% load Environment2 BW
% load Environment3 BW

mapMatrix = BW;
mapScale = 1;

scanAngles = -3*pi/8:pi/12:3*pi/8; % ángulos lidar
maxRange = 70; % rango
lidarNoiseVariance = 0.1^2;
lidarNoiseSeeds = randi(intmax, size(scanAngles));

% Parámetros de velocidad máxima
maxLinSpeed = 0.9; % velocidad lineal
maxAngSpeed = 0.9; % velocidad angular

% Ruta en el entorno 1
initX = 34; % punto de inicio en x
initY = 31; % punto de inicio en y
initTheta = pi/2; % ángulo inicial

fig = figure("Name", "BW");
set(fig, "Visible", "on");
ax = axes(fig);

show(binaryOccupancyMap(mapMatrix), "Parent", ax);
hold on
plotTransforms([initX, initY, 0], eul2quat([initTheta, 0, 0]), "MeshFilePath", "groundvehicle.stl", "View", "2D");
light;
hold off

% Configuración del modelo y entorno de simulación
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
env.ResetFcn = @(in)exampleHelperRLAvoidObstaclesResetFcn(in, scanAngles, maxRange, mapMatrix);
env.UseFastRestart = "On";

% Red del actor
statePath = [
    featureInputLayer(numObservations, "Normalization", "none", "Name", "State")
    fullyConnectedLayer(64, "Name", "ActorFC1")
    reluLayer("Name", "ActorRelu1")
    fullyConnectedLayer(64, "Name", "ActorFC2")
    reluLayer("Name", "ActorRelu2")];
meanPath = [
    fullyConnectedLayer(numActions, "Name", "ActionMeanFC")
    tanhLayer("Name", "ActionMean")];  % Use tanhLayer to ensure finite actions
stdPath = [
    fullyConnectedLayer(numActions, "Name", "ActionStdFC")
    softplusLayer("Name", "ActionStd")];

actorNetwork = layerGraph(statePath);
actorNetwork = addLayers(actorNetwork, meanPath);
actorNetwork = addLayers(actorNetwork, stdPath);
actorNetwork = connectLayers(actorNetwork, "ActorRelu2", "ActionMeanFC");
actorNetwork = connectLayers(actorNetwork, "ActorRelu2", "ActionStdFC");

% Crear el objeto rlContinuousGaussianActor
actorOptions = rlOptimizerOptions("LearnRate", 1e-4, "L2RegularizationFactor", 1e-4, "GradientThreshold", 1);
actor = rlContinuousGaussianActor(actorNetwork, obsInfo, actInfo, ...
    "ObservationInputNames", "State", ...
    "ActionMeanOutputNames", "ActionMean", ...
    "ActionStandardDeviationOutputNames", "ActionStd");

% Red del crítico
criticNetwork = [
    featureInputLayer(numObservations, "Normalization", "none", "Name", "State")
    fullyConnectedLayer(64, "Name", "CriticFC1")
    reluLayer("Name", "CriticRelu1")
    fullyConnectedLayer(64, "Name", "CriticFC2")
    reluLayer("Name", "CriticRelu2")
    fullyConnectedLayer(1, "Name", "Value")];

criticNetwork = dlnetwork(layerGraph(criticNetwork));

% Crear el objeto rlValueFunction
criticOptions = rlOptimizerOptions("LearnRate", 1e-3, "L2RegularizationFactor", 1e-4, "GradientThreshold", 1);
critic = rlValueFunction(criticNetwork, obsInfo, "ObservationInputNames", "State");

% Opciones del agente PPO
agentOpts = rlPPOAgentOptions(...
    "ClipFactor", 0.2, ...
    "EntropyLossWeight", 0.01, ...
    "MiniBatchSize", 64, ...
    "ExperienceHorizon", 512, ...
    "DiscountFactor", 0.995, ...
    "AdvantageEstimateMethod", "gae", ...
    "GAEFactor", 0.95, ...
    "SampleTime", sampleTime);

% Crear el agente PPO
obstacleAvoidanceAgent = rlPPOAgent(actor, critic, agentOpts);
open_system(mdl + "/Agent")

% Normalizar recompensas
function normReward = normalizeReward(reward)
    % Normaliza la recompensa a un rango cercano a 0
    normReward = reward / 1e6; % Ajuste este valor según sea necesario
end

% Añadir diagnósticos para verificar las acciones generadas
function verifyActions(actions)
    if any(~isfinite(actions))
        error('Se detectaron acciones no finitas: %s', mat2str(actions));
    end
end

% Función de recompensa modificada para normalización
function reward = exampleHelperRewardFunction(action, state)
    reward = -norm(action) - norm(state); % Modificar según el entorno
end

% Opciones de entrenamiento
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

doTraining = true; % Cambiar a true para entrenar

if doTraining
    % Entrenar al agente
    trainingStats = train(obstacleAvoidanceAgent, env, trainOpts);
else
    % Cargar un agente preentrenado
    load exampleHelperAvoidObstaclesAgent obstacleAvoidanceAgent
end

% Simulación y visualización de resultados
set_param("exampleHelperAvoidObstaclesMobileRobot", "StopTime", "850");
out = sim("exampleHelperAvoidObstaclesMobileRobot.slx");

for i = 1:5:size(out.range, 3)
    u = out.pose(i, :);
    r = out.range(:, :, i);
    exampleHelperAvoidObstaclesPosePlot(u, mapMatrix, mapScale, r, scanAngles, ax);
end

% Crear una figura para mostrar el mapa binario y la trayectoria
figure;
BW_inverted = imcomplement(BW); % Invertir los valores del mapa binario
imshow(BW_inverted); % Mostrar el mapa binario invertido como una imagen de fondo
hold on;

% Definir los límites del gráfico usando las dimensiones del mapa 2D
xLimits = [1 size(BW, 2)];
yLimits = [1 size(BW, 1)];
xlim(xLimits);
ylim(yLimits);

% Crear una matriz vacía para almacenar la trayectoria del robot
trajectory = [];

% Iterar a través de los datos de la trayectoria y trazar la trayectoria del robot
for i = 1:5:size(out.range, 3)
    u = out.pose(i, :);
    u(2) = (-1) * (u(2) - 256); % Ajustar las coordenadas del eje Y
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
