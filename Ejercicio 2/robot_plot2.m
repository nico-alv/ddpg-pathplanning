function VerticesA = robot_plot2(robot)

X = robot.x;
Y = robot.y;
Tita = robot.tita;

Vertices_aux = [];

Vertices = [1.22 0.75 0;
    -0.52 0.75 0;
    -0.52 -0.75 0;
    1.22 -0.75 0;
    1.22 0.75 0];
T_trans = [1 0 0 X;
        0 1 0 Y;
        0 0 1 0;
        0 0 0 1];
    T_rot = [cos(Tita) -sin(Tita) 0 0;sin(Tita) cos(Tita) 0 0;0 0 1 0;0 0 0 1];
for cont = 1:5
    P = Vertices(cont,:);
    M = T_trans*T_rot*[P(1) P(2) 0 1]';
    Vertices_aux(cont,:) = [M(1) M(2) 0];
end

plot(Vertices_aux(:,1),Vertices_aux(:,2),'-r','LineWidth',1);

VerticesA = [Vertices_aux(1,1) Vertices_aux(1,2) 0;
    Vertices_aux(2,1) Vertices_aux(2,2) 0;
    Vertices_aux(2,1) Vertices_aux(2,2) 0;
    Vertices_aux(1,1) Vertices_aux(1,2) 0;
    Vertices_aux(2,1) Vertices_aux(2,2) 0;
    Vertices_aux(3,1) Vertices_aux(3,2) 0;
    Vertices_aux(3,1) Vertices_aux(3,2) 0;
    Vertices_aux(2,1) Vertices_aux(2,2) 0;
    Vertices_aux(3,1) Vertices_aux(3,2) 0;
    Vertices_aux(4,1) Vertices_aux(4,2) 0;
    Vertices_aux(4,1) Vertices_aux(4,2) 0;
    Vertices_aux(3,1) Vertices_aux(3,2) 0;
    Vertices_aux(4,1) Vertices_aux(4,2) 0;
    Vertices_aux(1,1) Vertices_aux(1,2) 0;
    Vertices_aux(1,1) Vertices_aux(1,2) 0;
    Vertices_aux(4,1) Vertices_aux(4,2) 0];