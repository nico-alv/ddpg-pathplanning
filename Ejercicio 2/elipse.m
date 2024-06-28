function elipseGrid = elipse(media_grp,cov_grp)
anglElps = linspace(0,2*pi);
[eigenvec,eigenval] = eig(cov_grp);
lamda = diag(eigenval);
if(lamda(1)>lamda(2))
  largest_eigenval = lamda(1);
  smallest_eigenval = lamda(2);
  largest_eigenvec = eigenvec(:,1); % compute to get angle
else
  largest_eigenval = lamda(2); 
  smallest_eigenval = lamda(1);
  largest_eigenvec = eigenvec(:,2);
end
% calculo del angulo entre eje-x y el mayor eigvector
angle = atan2(largest_eigenvec(2), largest_eigenvec(1));
if(angle < 0)
angle = angle + 2*pi;
end

a = sqrt(largest_eigenval);
b = sqrt(smallest_eigenval);
% elipse en cordenadas x-y
ellipse_x_r  = 5*a*cos( anglElps );
ellipse_y_r  = 5*b*sin( anglElps );
% Rotacion de la matriz Cov
R = [cos(angle) sin(angle); -sin(angle) cos(angle)];
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
X0 = media_grp(1);
Y0 = media_grp(2);
elipseGrid = [r_ellipse(:,1)+X0 r_ellipse(:,2)+Y0];