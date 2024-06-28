function Lineas = GenerarLineasyVertices(Vertices, Caras)

[m,n] = size(Vertices);
[s,r] = size(Caras);

Lineas = [];

for contFilas = 1:s
    for contCol = 1:(r - 1)
        alfa = [];
        beta = [];
        a = Caras(contFilas,contCol);
        b = Caras(contFilas,contCol + 1);
        if Vertices(a,2) == Vertices(b,2)
            alfa = 0;
            beta = Vertices(a,2);
        end
        if Vertices(a,1) == Vertices(b,1)
            alfa = 100000;
            beta = Vertices(a,1);
        end
        if Vertices(a,1) ~= Vertices(b,1)
            alfa = (Vertices(a,2) - Vertices(b,2))/(Vertices(a,1) - Vertices(b,1));
            beta = -alfa*Vertices(b,1) + Vertices(b,2);
        end
        Lineas = [Lineas; [Vertices(a,1) Vertices(b,1) alfa; Vertices(a,2) Vertices(b,2) beta]];
    end
    a = Caras(contFilas,r);
    b = Caras(contFilas,1);
    if Vertices(a,2) == Vertices(b,2)
        alfa = 0;
        beta = Vertices(a,2);
    end
    if Vertices(a,1) == Vertices(b,1)
        alfa = 100000;
        beta = Vertices(a,1);
    end
    if Vertices(a,1) ~= Vertices(b,1)
        alfa = (Vertices(a,2) - Vertices(b,2))/(Vertices(a,1) - Vertices(b,1));
        beta = -alfa*Vertices(b,1) + Vertices(b,2);
    end
    Lineas = [Lineas; [Vertices(a,1) Vertices(b,1) alfa; Vertices(a,2) Vertices(b,2) beta]];
end