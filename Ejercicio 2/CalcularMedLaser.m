function medicion = CalcularMedLaser(Lineas)

global maxRange;
[m,n] = size(Lineas);

NumeroLineas = m/2;
laser = maxRange + 1;
distancia = maxRange;

for cont = 1:NumeroLineas
    alfa = Lineas(cont*2 - 1, 3);
    beta = Lineas(cont*2, 3);
    P1 = Lineas(cont*2 - 1:cont*2,1);
    P2 = Lineas(cont*2 - 1:cont*2,2);
   
    if alfa == 100000
        distancia = beta;
    elseif abs(alfa) < 0.00000000001
        distancia = maxRange;
    else
        distancia = -beta/alfa;
    end
    if (distancia < laser)&&(distancia >= 0)&&(sign(P1(2))*sign(P2(2)) == -1)
        laser = distancia;
    end
end

if (laser >= 0)&&(laser <= maxRange*0.998)
    medicion = laser;
else
    medicion = maxRange;
end