%Esta funcion plotea el laser dentro del ambiente
%Ingresan las coordenadas x,y del sistema de coordenadas global del entorno
%           Laser = coordenadas globales de las mediciones
%           Robot = Posicion del robot, en estructura x,y,tita
%           H = handle del laser
%--------------------------------------------------------------------------

function H = plotLaser(Laser,Robot)

Laser_1 = [];

orientacion_base = Robot.tita - pi/2;
if (orientacion_base < 0)
        orientacion_base = 2*pi*(1 + floor(abs(orientacion_base)/(2*pi))) + orientacion_base;
end
if (orientacion_base > 2*pi)
        orientacion_base = orientacion_base - 2*pi*floor(orientacion_base/(2*pi));
end

orientacion_base = floor(orientacion_base*180/pi);
limit_inf = orientacion_base;

for cont = 1:181
    angulo = (Laser(2,cont) + limit_inf)*pi/180;
    Laser_1 = [Laser_1 [Laser(1,cont)*cos(angulo) + Robot.x;Laser(1,cont)*sin(angulo) + Robot.y]];
end

Vertices = [Laser_1' zeros(181,1)];

Caras = [1:181];
Colores = [1 1 0];
H = patch('Faces',Caras,'Vertices',Vertices,'FaceVertexCData',Colores,...
      'FaceColor',[0.8 0.8 0.8],'Marker','*','MarkerSize',3,'MarkerEdgeColor','r','FaceAlpha',.3);