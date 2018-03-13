function [gamma,H,Y,R,PCA_GD,PCA_MJ]= data_associate_known(z,idft)
%function [zf,idf,zn, table]= data_associate_known(x,z,idz, table)
%
% For simulations with known data-associations, this function maintains
% a feature/observation lookup table. It returns the updated table, the
% set of associated observations and the set of observations to new features.

global PX PARAMS

if PARAMS.P_D ~= 1, error('P_D has to be 1 for the known DA'), end;

Nz= size(z,2);

PCA_GD= 1;
PCA_MJ= 1;

h= zeros(2*Nz,1);
H= zeros(2*Nz,3);

for i=1:Nz
    
    ii= (2*i-1:2*i);
    
    [h(ii), H(ii,:)]= lm_model(z(:,i), PARAMS.R, idft(i));
    
end
gamma= z(:) - h;

R= kron(eye(Nz),PARAMS.R);
Y= H*PX*H' + R;



function [h, H]= lm_model(z, R, lm_id)
global XX PX LM

% return normalised innovation squared (ie, Mahalanobis distance) and normalised distance

% auxiliary values
dx= LM(1,lm_id)  -XX(1); 
dy= LM(2,lm_id) - XX(2);
d2= dx^2 + dy^2;
d= sqrt(d2);

xd= dx/d;
yd= dy/d;
xd2= dx/d2;
yd2= dy/d2;

% predict z
h= [d;
    atan2(dy,dx) - XX(3)];

% Innovation vector
v= z-h; v(2)= pi_to_pi(v(2));

% Quick check to discard associations
% if abs(v(1)) > 2 || abs(v(2)) > deg2rad(15), nis= inf; return, end;


% calculate H
H = [-xd -yd 0; 
      yd2 -xd2 -1];

% Innovation vector covariance
S= H*PX*H' + R; 











