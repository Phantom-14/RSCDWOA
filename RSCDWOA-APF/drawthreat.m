function k = addThreat(x1,y1,z1,R)
global Height
%obstacles
%Set grid number
global scale;
n=20; 
theta = (-n:2:n)/n*pi;
phi = ([0,0:2:n])'/n*pi/2;
cosphi = cos(phi); cosphi(1) = 0; cosphi(end) = 0;
sintheta = sin(theta); sintheta(1) = 0; sintheta(end) = 0;
x = x1+R*cosphi*cos(theta);
y = y1+R*cosphi*sintheta;
z = z1+R*sin(phi)*ones(1,n+1)+0.1;

%The figure color is beach color
% colormap([0.77 0.61 0.4]);
meshz(y,x,z,'edgecolor','r');

% surf(x,y,z);
%Set title, etc
end

