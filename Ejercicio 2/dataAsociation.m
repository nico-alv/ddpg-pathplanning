function MAPA = dataAsociation(MedPost,MAPA)
[lMapa c] = size(MAPA);
[lpostes c] = size(MedPost);
Th = 0.8;
etiqueta = 1;
newPoste = [];
d = [];
% Get distance Map and Characteristics
for i = 1:lpostes
    for j = 1:lMapa
        d(j) = norm(MedPost(i,:)-MAPA(j,:));
        if d(j)<Th
            MAPA(j,:) = 0.5*(MedPost(i,:)+MAPA(j,:)); %Update means
        end
    end
    % identify New Point on Map
    if min(d)<Th
    else
        newPoste = [newPoste;MedPost(i,:)];
    end
    d = [];
end
if(~isempty(newPoste))
    MAPA = [MAPA;newPoste];
else
    MAPA = MAPA;
end
  