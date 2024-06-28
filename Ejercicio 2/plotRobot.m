function [graph_robot,Vertices, Caras] = plotRobot(robot)

X = robot.x;
Y = robot.y;
Tita = robot.tita;

Vertices_aux = [];

Vertices = [0.22 0.1 0;
    0.1 0.19 0;
    -0.1 0.19 0;
    -0.22 0.1 0;
    -0.22 -0.1 0;
    -0.1 -0.19 0;
    0.1 -0.19 0;
    0.22 -0.1 0;
    0.02 -0.1 0;
    0.11 -0.1 0;
    0.11 -0.05 0;
    0.18 -0.05 0;
    0.18 0.05 0;
    0.11 0.05 0;
    0.11 0.1 0;
    0.02 0.1 0];

M = length(Vertices(:,1)); %este valor puede ser fijo
T_trans = [1 0 0 X;
        0 1 0 Y;
        0 0 1 0;
        0 0 0 1];
    T_rot = [cos(Tita) -sin(Tita) 0 0;sin(Tita) cos(Tita) 0 0;0 0 1 0;0 0 0 1];
for cont = 1:M
    P = Vertices(cont,:);
    M = T_trans*T_rot*[P(1) P(2) 0 1]';
    Vertices_aux(cont,:) = [M(1) M(2) 0];
end
    
Caras = [1 2 3 4 5 6 7 8 1 NaN NaN NaN;
    9 10 11 12 13 14 15 16 9 NaN NaN NaN];

Colores = [0.7 0.7 0.7;1 1 1];

graph_robot = patch('Faces',Caras,'Vertices',Vertices_aux,'FaceVertexCData',Colores,...
      'FaceColor','flat');