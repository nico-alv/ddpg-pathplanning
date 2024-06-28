function Caracteristicas = EncontrarCaracteristicas(Laser,robot)
global maxRange;
Cart = [];
Postes = [];
for j=1:length(Laser)
    if(Laser(1,j)<0.9*maxRange)
        Cart = [Cart;Laser(1,j)*cos(Laser(2,j)*pi/180-pi/2) Laser(1,j)*sin(Laser(2,j)*pi/180-pi/2)];
    end
end 
if(~isempty(Cart))
    for cont=1:length(Cart(:,1))
        x = Cart(cont,1);
        y = Cart(cont,2);
        xx = x*cos(robot.tita) - y*sin(robot.tita) + robot.x;
        yy = x*sin(robot.tita) + y*cos(robot.tita) + robot.y;
        Postes = [Postes; xx yy ];
    end
end
[f,c] = size(Postes);
Caracteristicas = [Postes ones(f,1)];
