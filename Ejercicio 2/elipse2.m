function elipseVector = elipse(media_grp,cov_grp)
% example: media_grp in R2(x,y), cov_grp in M2(x1,x2:x3,x4)

anglElps = linspace(0,2*pi);
[eigenvec,eigenval] = eig(cov_grp);
lamda = diag(eigenval);

% gets the largest eigenvalue
if(lamda(1)>lamda(2))
  largest_eigenval = lamda(1);
  smallest_eigenval = lamda(2);
  largest_eigenvec = eigenvec(:,1); % compute to get angle
else
  largest_eigenval = lamda(2); 
  smallest_eigenval = lamda(1);
  largest_eigenvec = eigenvec(:,2);
end 

% compute the angel between x-axis and the largest eigenvalue
angle = atan2(largest_eigenvec(2), largest_eigenvec(1));
if(angle < 0)
angle = angle + 2*pi;
end

a = abs(largest_eigenval);
b = abs(smallest_eigenval);
% elipse in coords x-y
%assume an amplifier factor 50 to see bigger
ellipse_x_r  = 50*a*cos( anglElps );
ellipse_y_r  = 50*b*sin( anglElps );
% Rotation of cov matriz
R = [cos(angle) sin(angle); -sin(angle) cos(angle)];
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
X0 = media_grp(1);
Y0 = media_grp(2);
elipseVector = [r_ellipse(:,1)+X0 r_ellipse(:,2)+Y0];