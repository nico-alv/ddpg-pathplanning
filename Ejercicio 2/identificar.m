function MapLabel = identificar(Caracteristicas,Thdist)
etiqueta = 1;
MapT=[];
[lenCar,cc] = size(Caracteristicas);
if (~isempty(Caracteristicas))
  for j=1:lenCar-1
      vect = Caracteristicas(j,1:2)- Caracteristicas(j+1,1:2);
      if norm(vect)<Thdist
          CarLabel = [Caracteristicas(j,1:2) etiqueta];
          MapT = [MapT;CarLabel];
      else
          CarLabel = [Caracteristicas(j,1:2) etiqueta];
          MapT = [MapT;CarLabel];
          etiqueta = etiqueta+1;
      end
  end
  CarLabel = [Caracteristicas(lenCar,1:2) etiqueta];
  MapT = [MapT;CarLabel];
  MapLabel = MapT;
else
    MapLabel = []; 
end
 