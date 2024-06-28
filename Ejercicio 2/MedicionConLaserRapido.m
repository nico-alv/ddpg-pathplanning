 function Laser = MedicionConLaserRapido(Vertices, Caras, Robot)

global maxRange;

Laser = [];
[m,n] = size(Vertices);
limiteInf = Robot.tita - pi/2;
distancia = -1000;

for cont = 0:180
    angulo = (cont)*pi/180 + limiteInf;
    VerticesRotados = RotacionDeVertices2(Vertices, Robot, angulo);
    Lineas = GenerarLineasyVertices(VerticesRotados, Caras);
    distancia = CalcularMedLaser(Lineas) + 0.0001*randn(1,1); % Ruido de la medicion
    Laser = [Laser [distancia;cont]];
end

% [m,n] = size(VerticesRotados);
% figure(2);
% hold on
% for cont = 1:m
%     plot(VerticesRotados(cont,1), VerticesRotados(cont,2),'x');
% end
% floor((angulo - pi/2)*180/pi)