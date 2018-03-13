
function PnMA= prob_nMA(xx,Px)

global LM PARAMS
 
% % calculate how much we need to include in the EFOV
% EFVOx= norminv(1 - PARAMS.I_FOV/2, 0, sqrt(Px(1,1)));
% EFVOy= norminv(1 - PARAMS.I_FOV/2, 0, sqrt(Px(2,2)));
% EFOV= sqrt(EFVOx^2 + EFVOy^2); 
lambda_FOV= max(eig(Px(1:2,1:2)));
EFOV= -lambda_FOV * norminv(PARAMS.I_FOV/4,0,1);
% EFOV= lambda_FOV * norminv(1 - PARAMS.I_FOV/2,0,1);

% Get all visible landmarks, assuming no mis-extractions here
idf= get_visible_landmarks(xx,PARAMS.maxRange+EFOV, 0);

lm= LM(:,idf);
n_L= length(idf);

if n_L < 2, PnMA= 1; return, end;

% create table with yn
y2= NaN(n_L);
M= cell(n_L);
H= cell(n_L,1);

yStarY= ones(1,n_L).*inf;
y2StarY= zeros(1,n_L);
boundSqrtTerm= sqrt( chi2inv(1 - PARAMS.I_y/n_L, PARAMS.m_F) );
for t= 1:n_L
    
    [h_t,H_t]= compute_lm_model(lm(:,t));
    Y= H_t * Px * H_t' + PARAMS.R;
    
    for l= 1:n_L
        if t == l, continue, end;
        
        [h_l,H_l]= compute_lm_model(lm(:,l));
        
        y= h_t - h_l;
        M= (H_t - H_l)*Px*(H_t - H_l)';
        sqrtM= sqrtm(M);
        lambda2= min( eig(sqrtM /Y * sqrtM) );
        y2= y'/M*y;
        
        yStarM= sqrt(y2) - boundSqrtTerm;
        
        yStarY_current= yStarM * sqrt(lambda2);
        if yStarY_current < yStarY(t),
            yStarY(t)= yStarY_current;
        end
    end
    
    % Eliminate negative values
    if yStarY(t) > 0, y2StarY(t)= yStarY(t)^2; end;
end

% Integrity risk
PnMA= 0;
for t= 1:n_L
    PnMA= PnMA + chi2cdf(0.25*y2StarY(t),5);
end
PnMA= (1 - PARAMS.I_y/n_L)*PnMA - (n_L - 1) - PARAMS.I_FOV;

% Eliminate negative values;
PnMA= max(PnMA,0);







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [h,H]= compute_lm_model(lm)

global XX

dx= lm(1) - XX(1);
dy= lm(2) - XX(2);
d2= dx^2 + dy^2;
d= sqrt(d2);

% calculate h
h= [d; atan2(dy,dx) - XX(3)];

% calculate H
H = [-dx/d, -dy/d,  0;
      dy/d2, -dx/d2, -1];
















































