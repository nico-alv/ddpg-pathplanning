load Environment3 BW

mapMatrix = BW;
mapScale = 1;

scanAngles = -3*pi/8:pi/24:3*pi/8; %angulos lidar
%-3*pi/8:pi/8:3*pi/8
maxRange = 25;%range
lidarNoiseVariance = 0.1^2;
lidarNoiseSeeds = randi(intmax,size(scanAngles));

% Max speed parameters
maxLinSpeed = 0.9; %lineal speed
maxAngSpeed = 0.9; %angular speed

%%%%route1envieronment1%%%%%
initX = 135; %start point x
initY =27; %start point y
%%%%%%%%%%%%%
initTheta = pi;%ang init

% initX = 121; %start point x
% initY =91; %start point y
% initTheta = pi/2;%ang init

fig = figure("Name","BW");
set(fig, "Visible","on");
ax = axes(fig);

show(binaryOccupancyMap(mapMatrix),"Parent",ax);
hold on
plotTransforms([initX,initY,0],eul2quat([initTheta,0,0]),"MeshFilePath","groundvehicle.stl","View","2D");
light;
hold off

mdl = "exampleHelperAvoidObstaclesMobileRobot";
Tfinal = 1000;
sampleTime = 0.1;

agentBlk = mdl + "/Agent";

open_system(mdl)

open_system(mdl + "/Environment")

numObservations = length(scanAngles);
obsInfo = rlNumericSpec([numel(scanAngles) 1],...
    "LowerLimit",zeros(numel(scanAngles),1),...
    "UpperLimit",ones(numel(scanAngles),1)*maxRange);
numObservations = obsInfo.Dimension(1);

numActions = 2;
actInfo = rlNumericSpec([numActions 1],...
    "LowerLimit",-1,...
    "UpperLimit",1);

env = rlSimulinkEnv(mdl,agentBlk,obsInfo,actInfo);
env.ResetFcn = @(in)exampleHelperRLAvoidObstaclesResetFcn(in,scanAngles,maxRange,mapMatrix);
env.UseFastRestart = "On";

statePath = [
    featureInputLayer(numObservations, "Normalization","none","Name","State")
    fullyConnectedLayer(50,"Name","CriticStateFC1")
    reluLayer("Name","CriticRelu1")
    fullyConnectedLayer(25,"Name","CriticStateFC2")];
actionPath = [
    featureInputLayer(numActions,"Normalization","none","Name","Action")
    fullyConnectedLayer(25,"Name","CriticActionFC1")];
commonPath = [
    additionLayer(2,"Name","add")
    reluLayer("Name","CriticCommonRelu")
    fullyConnectedLayer(1,"Name","CriticOutput")];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,"CriticStateFC2","add/in1");
criticNetwork = connectLayers(criticNetwork,"CriticActionFC1","add/in2");
criticNetwork = dlnetwork(criticNetwork);

criticOptions = rlOptimizerOptions("LearnRate",1e-3,"L2RegularizationFactor",1e-4,"GradientThreshold",1);
critic = rlQValueFunction(criticNetwork,obsInfo,actInfo,"ObservationInputNames","State","ActionInputNames","Action");

actorNetwork = [
    featureInputLayer(numObservations,"Normalization","none","Name","State")
    fullyConnectedLayer(50,"Name","actorFC1")
    reluLayer("Name","actorReLU1")
    fullyConnectedLayer(50, "Name","actorFC2")
    reluLayer("Name","actorReLU2")
    fullyConnectedLayer(2, "Name","actorFC3")
    tanhLayer("Name","Action")];
actorNetwork = dlnetwork(actorNetwork);

actorOptions = rlOptimizerOptions("LearnRate",1e-4,"L2RegularizationFactor",1e-4,"GradientThreshold",1);
actor = rlContinuousDeterministicActor(actorNetwork,obsInfo,actInfo);
%Acelerar redes neuronales
critic.UseDevice = "gpu";
actor.UseDevice = "gpu";

agentOpts = rlDDPGAgentOptions(...
    "SampleTime",sampleTime,...
    "ActorOptimizerOptions",actorOptions,...
    "CriticOptimizerOptions",criticOptions,...
    "DiscountFactor",0.995, ...
    "MiniBatchSize",128, ...
    "ExperienceBufferLength",1e6); 

agentOpts.NoiseOptions.Variance = 0.1;
agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;

obstacleAvoidanceAgent = rlDDPGAgent(actor,critic,agentOpts);
open_system(mdl + "/Agent")

maxEpisodes = 2000;
maxSteps = ceil(Tfinal/sampleTime);
trainOpts = rlTrainingOptions(...
    "MaxEpisodes",maxEpisodes, ...
    "MaxStepsPerEpisode",maxSteps, ...
    "ScoreAveragingWindowLength",50, ...
    "StopTrainingCriteria","AverageReward", ...
    "StopTrainingValue",99999, ...
    "Verbose", true, ...
    "Plots","training-progress", ...
    "UseParallel", true);
trainOpts.ParallelizationOptions.Mode = "async";

doTraining = true; % Toggle this to true for training. 

if doTraining
    % Train the agent.
    trainingStats = train(obstacleAvoidanceAgent,env,trainOpts);
else
    % Load pretrained agent for the example.
    load exampleHelperAvoidObstaclesAgent obstacleAvoidanceAgent
end

set_param("exampleHelperAvoidObstaclesMobileRobot","StopTime","1000");
out = sim("exampleHelperAvoidObstaclesMobileRobot.slx");

for i = 1:5:size(out.range,3)
   u = out.pose(i,:);
   r = out.range(:,:,i);
   exampleHelperAvoidObstaclesPosePlot(u,mapMatrix,mapScale,r,scanAngles,ax);
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
for i = 1:5:size(out.range,3)
    u = out.pose(i,:);
    % Ajustar las coordenadas del eje Y
    u(2) = (-1) * (u(2) - 256);
    % Aquí puedes realizar cualquier otra operación necesaria con 'u'
    trajectory = [trajectory; u];
end

% Trazar la trayectoria en el mapa 2D
plot(trajectory(:,1), trajectory(:,2), 'r-', 'LineWidth', 2);

% Etiquetas y título
xlabel('Eje X');
ylabel('Eje Y');
title('Mapa Binario con Trayectoria del Robot');

% Marcar el punto de inicio con color verde
startPoint = trajectory(1,:);
plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Marcar el punto de termino con color rojo
endPoint = trajectory(end,:);
plot(endPoint(1), endPoint(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Mostrar la leyenda
legend('Trayectoria', 'Inicio', 'Termino');

% Habilitar la cuadrícula


grid on;

% Desactivar el modo de espera para permitir que la figura se muestre correctamente
drawnow;