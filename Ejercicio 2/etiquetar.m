function Media_Label = etiquetar(Media_Label,MapaIn,MAPA)
[lMapaIn c] = size(MapaIn);
[lMAPA c] = size(MAPA);
Th = 0.8;
etiqueta = 1;
d = [];
% Get distance Map and Characteristics
for i = 1:lMapaIn 
    for j = 1:lMAPA
        d(j) = norm(MapaIn(i,:)-MAPA(j,:));
    end
    [val idx] = min(d);
    
    if(val<Th)
        Media_Label = [Media_Label;[MapaIn(i,:) idx]];
    else
        Media_Label = [Media_Label;[MapaIn(i,:) lMAPA+1]];
        disp('new')
    end
    
end
