function VerticesRotados = RotacionDeVertices2(Vertices, robot, angulo)

[m,n] = size(Vertices);
Vertices2 = [];
%Rotacion de los vertices al sistema de coordenadas del robot
beta = angulo;
for cont = 1:m
    Vertices2 = [Vertices2; ([cos(beta) sin(beta); -sin(beta) cos(beta)]*[Vertices(cont,1)- robot.x; Vertices(cont,2) - robot.y])'];
end;

VerticesRotados = Vertices2;