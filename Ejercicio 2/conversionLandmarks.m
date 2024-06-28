function [Caras,Vertices] = conversionLandmarks(M)

Caras = [];
Vertices = [];
paso = 0.2;

for cont = 1:length(M(:,1))
    x = M(cont,1);
    y = M(cont,2);
    
    Vertices = [Vertices; [x-paso y-paso 0;x-paso y+paso 0;x+paso y+paso 0;x+paso y-paso 0]];
    Caras = [Caras;[(cont-1)*4+1 (cont-1)*4+2 (cont-1)*4+3 (cont-1)*4+4 (cont-1)*4+1]];
end