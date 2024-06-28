% ******************** MAPPING WITH FEATURE  COMPARITION ******************
%** Autor: JAVIER PRADO R.                                               **
%** Fecha: 06/06/2015
%**************************************************************************

figure(1), hold on
clear all,
clc,
xlim([-5 10]);
ylim([-5 10]);
global maxRange;
maxRange = 8; %Distancia máxima del laser
% axis equal

pasoTiempo = 0.1; %Tiempo de Sampling de mi sistema

%Inicializo Robots

robot.x = 0;
robot.y = 0; %es el robot segun odometria
robot.tita = 0;

grid 
robot2 = robot; %es el robot real
%ambos robots empiezan en el mismo lugar

%Ubico landmarks. Con tecla enter, finalizo ubicación. Landmarks son
%guaradadas en matriz M, que contiene las posiciones reales [x,y] de cada
%landmark.

M = ginput();
plot(M(:,1),M(:,2),'ob'); %grafico landmarks
[Caras,Vertices] = conversionLandmarks(M);

%Genero el camino que quiero que siga el robot, entre las landmarks. El
%camino solo va de izquierda a derecha

[X,Y] = ginput;
xmax = max(X); xmin = min(X);
xx = xmin:0.01:xmax;
yy = interp1(X,Y,xx,'spline');
CaminoReferencia = [xx' yy'];
plot(xx',yy','-r');

%Inicializo Robots en el punto de partida del camino generado

robot.x = X(1);
robot.y = Y(1);
robot.tita = atan2(Y(2) - Y(1), X(2) - X(1)); %es la pose del robot según la odometría. Sin embargo, tener presente que el
%robot "cree" que tiene su pose sin error.

robot2 = robot; %es el robot real
%ambos robots empiezan en el mismo lugar

%Grafico ambos robots (odométrico y real)
H1 = plotRobot(robot);
H2 = plotRobot2(robot2);
Thdist = 0.9;
etiqueta = 1;
%Genero un bucle para controlar el camino del robot
MapT = []; X0 = []; Y0 = []; Larg_lamda = []; Small_lamda = []; Alfa = []; Elipses = [];
MAPA = [];
MAPA_Label = [];
MAPA_old = [];
Media_Label = [];
NumPostReal = 0;
%%
for cont = 2:length(xx)-1
    delete(H1);
    delete(H2);
    
    %Sensing Characteristics
    Laser = MedicionConLaserRapido(Vertices, Caras, robot2);
     
    H3 = plotLaser(Laser,robot2); 
    BuscoPostes = [];  
    Caracteristicas = EncontrarCaracteristicas(Laser,robot2);
    if(~isempty(Caracteristicas))
        plot(Caracteristicas(:,1),Caracteristicas(:,2),'gx');
    end  
 %%   
    % Indentifying Characteristics by Euclidian Distance. 
    MapLabel = identificar(Caracteristicas,Thdist);
    if (~isempty(MapLabel))
    NumPost = max(MapLabel(:,3));
    MedPost = media_inst(NumPost,MapLabel); % Medias Etiquetadas
    end
    
    if(~isempty(MAPA))
        MAPA = dataAsociation(MedPost,MAPA);
    else
        
        MAPA = MedPost; 
    end
    if(~isempty(MedPost) && ~isempty(Media_Label))
        Media_Label = etiquetar(Media_Label,MedPost,MAPA);
    else
        Media_Label = identificar(MedPost,Thdist);
    end  
    MAPA_old = MAPA;   
    
    % Covariance compute for every iteration and group.
    NumPost = max(Media_Label(:,3));
    for ng=1:NumPost
        seleccion = find(Media_Label(:,3) == ng);
        grp = Media_Label(seleccion',:);
        [f,c] = size(grp);
        if f>2 % Condition to get Covariance matrix
          NumPostReal = NumPostReal+1; 
          media_grp =  mean(grp(:,1:2));
          diffOdo_x = abs(robot.x-robot2.x);
          diffOdo_y = abs(robot.y-robot2.y);
          diffOdo_tita = abs(robot.tita-robot2.tita);
          odometria = [diffOdo_x;diffOdo_y;diffOdo_tita];
          cov_grp = cov(grp(:,1:2))+cov(odometria)
          elipseGrid = elipse(media_grp,cov_grp);
          plt_eli(NumPostReal) = plot(elipseGrid(:,1),elipseGrid(:,2),'-b','LineWidth',1);
        end
    end  
%%
    M = MAPA;
    
    plt_media = plot(M(:,1),M(:,2),'m*','LineWidth',2); % Media al tiempo k.
    titulo = strcat('Número de postes estimados: ',num2str(size(MAPA,1)));
    title(titulo)
    % Controller reference
    Q = .01*eye(3,3);
    H = [cos(robot.tita) 0;sin(robot.tita) 0; 0 1];
    xref = xx(cont); yref = yy(cont);
    titaref = atan2(yy(cont+1) - robot.y,(xx(cont+1) - robot.x));
    deltax = (xref-robot.x)/pasoTiempo + 0.1*(robot.x - xx(cont-1))/pasoTiempo;
    deltay = (yref-robot.y)/pasoTiempo + 0.3*(robot.y - yy(cont-1))/pasoTiempo;
    sigma1 = (Q(1,1))^2;
    sigma2 = (Q(2,2))^2;
    sigma3 = (Q(3,3))^2;
    Control(2) = (titaref-robot.tita)/pasoTiempo;
    Control(1) = (2*deltax*sigma2^2*cos(robot.tita))/(sigma2^2*cos(2*robot.tita) - sigma1^2*cos(2*robot.tita) + sigma1^2 + sigma2^2) + (2*deltay*sigma1^2*sin(robot.tita))/(sigma2^2*cos(2*robot.tita) - sigma1^2*cos(2*robot.tita) + sigma1^2 + sigma2^2);
    if Control(1) > 0.3
        Control(1) = 0.3;
    end
    if abs(Control(2)) > 1
        if (Control(2) <= 0)
            Control(2) = -1;
        else
            Control(2) = 1;
        end
    end
    V = Control(1);
    W = Control(2);
    robot.x = robot.x + pasoTiempo*V*cos(robot.tita);
    robot.y = robot.y + pasoTiempo*V*sin(robot.tita);
    robot.tita = robot.tita + pasoTiempo*W;
    
    robot2.x = robot2.x + pasoTiempo*V*cos(robot2.tita)+0.003*rand(1,1);
    robot2.y = robot2.y + pasoTiempo*V*sin(robot2.tita)+0.003*rand(1,1);
    robot2.tita = robot2.tita + pasoTiempo*W+0.02*rand(1,1)*W;
    
    H1 = plotRobot(robot);
    H2 = plotRobot2(robot2);
    pause(pasoTiempo);
    delete(H3);
    %delete(plt_media);
     for kkk=1:NumPostReal
      delete(plt_eli(kkk));
     end
    NumPostReal = 0;
end
%% **********************  RESULTADOS AL FINAL DE LA SIMULACION ******************************
% ENTREGO MATRIZ DE MAPA M: Y SU COVARIANZA AL TERMINO DE LA TRAYECTORIA
figure(2)
M
for ng=1:NumPost
        seleccion = find(Media_Label(:,3) == ng);
        grp = Media_Label(seleccion',:);
        [f,c] = size(grp);
        if f>2 % Condition to get Covariance matrix
          NumPostReal = NumPostReal+1; 
          media_grp =  mean(grp(:,1:2));
          cov_grp = cov(grp(:,1:2));
          diffOdo_x = abs(robot.x-robot2.x);
          diffOdo_y = abs(robot.y-robot2.y);
          diffOdo_tita = abs(robot.tita-robot2.tita);
          odometria = [diffOdo_x;diffOdo_y;diffOdo_tita];
          cov_grp = cov(grp(:,1:2))+cov(odometria);
          elipseGrid = elipse(media_grp,cov_grp);
          
          plot(Media_Label(:,1),Media_Label(:,2),'.r')
          hold on
          plt_eli(NumPostReal) = plot(elipseGrid(:,1),elipseGrid(:,2),'-b','LineWidth',1);
          hold on
          plot(M(:,1),M(:,2),'g*')
          hold on
        end
        title('Media y Covarianza para cada Characteristica Estimada')
        legend('Estimados por cada t','Covarianza','MAPA')
        grid
end  
