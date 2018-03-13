
function [idf]= data_associate_LNN (z)

global hlm Hlm PARAMS LM

Nz= size(z,2);
Nlm= size(LM,2);

% Initialize associations
idf= zeros(1,Nz);

% Initialize lm models to be filled in the next function
hlm= cell(Nlm,1);
Hlm= cell(Nlm,1);

% Create T associations table where the each columns corresponds to a lm
% (not a msmt as before)
IIVN= associations_tables(z, Nz, Nlm, PARAMS.R, PARAMS.gate);


% Best association by rows (by feature)
for i=1:Nz
    validated_assoc= IIVN(i,:) ~= 0;
    if sum(validated_assoc) > 1
        [chosenValue,chosenInd]= min(IIVN(i,validated_assoc));
        inds= find(validated_assoc);
        chosenInd= inds(chosenInd);
        IIVN(i,validated_assoc)= 0;
        IIVN(i,chosenInd)= chosenValue;
    end
end

% Best association by columns (by lm)
for l=1:Nlm
    validated_assoc= IIVN(:,l) ~= 0;
    sum_validated_assoc= sum(validated_assoc);
    if sum_validated_assoc == 1
        idf(find(validated_assoc))= l;
        if sum_validated_assoc > 1
            [chosenValue,chosenInd]= min(IIVN(validated_assoc,l));
            inds= find(validated_assoc);
            chosenInd= inds(chosenInd);
            idf(chosenInd)= l;
            
            IIVN(validated_assoc,l)= 0;
            IIVN(chosenInd,l)= chosenValue;
        end
    end
end



% % create the models for the association
% h= []; H= [];
% for i= 1:Nz
%     if any(IIVN(i,:)) ~= 0
%         [~,chosenInd]= max(IIVN(i,:));
%         H= [H; Hlm{chosenInd}];
%         h= [h; hlm{chosenInd}];
%     end
% end
% 
% % Compute weighted norms
% gamma= z(:) - h;
% gamma(2:2:end)= pi_to_pi(gamma(2:2:end));
% Y= H*PX*H' + R;
% ngamma= gamma'*(Y\gamma);
% 
% % Update the estimate
% K= PX*H'/Y;
% PX= PX - K*H*PX;
% XX= XX + K*gamma;

    

    

% 
% 
% % Number of outliers in current associations
% phi= Nz - numLmAssoc;
% dof= PARAMS.dz .* (Nz - phi);
% psi_z= size(T_z,1);
% ngamma= zeros(psi_z,1); % initialize ngamma
% 
% % For the lm selected, select best association
% h= cell(psi_z,1);
% H= cell(psi_z,1);
% gamma= cell(psi_z,1);
% ny= zeros(psi_z,1);
% for j= 1:psi_z
%     
%     % Eliminate the associated msmts
%     zj= z(:, T_z(j,1:end) ~= 0 );
%     % make R a block diagonal with higher dimension
%     R= kron(eye(Nz-phi),PARAMS.R);
%     
%     % create the models for each association
%     for i= 1:Nz
%         if T_z(j,i) ~= 0
%             H{j}= [H{j};Hlm{T_z(j,i)}];
%             h{j}= [h{j};hlm{T_z(j,i)}];
%         end
%     end
%     
%     % Compute weighted norms
%     gamma{j}= zj(:) - h{j};
%     gamma{j}(2:2:end)= pi_to_pi(gamma{j}(2:2:end));
%     Y{j}= H{j}*PX_all*H{j}' + R;
%     ngamma(j)= gamma{j}'*(Y{j}\gamma{j});
%     
%     % Lower bound noncentrality parameters
%     ny(j)= interp1( 2:1:200, squeeze(LB(1,dof/2,:)), ngamma(j),'linear','extrap');
%     
% end
% 
% % Select association
% [~,jstar]= min(ngamma);
% ny(jstar)= [];
% ny_min= min(ny); % Min non-centrality parameter lower bound
% 
% % Check consistency of the nis
% if ngamma(jstar) < chi2inv(0.99, numLmAssoc*PARAMS.dz) || ~SWITCH.consistency
%     
%     % Calculate the P(CA)
%     if psi_z > 1
%         PCA= chi2cdf(0.25* ny_min , 3 +  dof ) - PARAMS.I_y - PARAMS.I_c;
%     else PCA= 1 - PARAMS.I_c;
%     end
%     
%     % Update the covariance to calculate P(HMI)
%     K= PX_all*H{jstar}'/Y{jstar};
%     PX_all= PX_all - K*H{jstar}*PX_all;
%     XX_all= XX_all + K*gamma{jstar};
%     
%     % Calculate P(HMI)
%     PHMI_CA= PHMI_with_CA(XX_all,PX_all(1:2,1:2), PARAMS.alert_limit);
% else     % Innovation doesn't pass the test -> Do Not Use!
%     PCA= 0;
%     PHMI_CA= PHMI_with_CA(XX_all,PX_all(1:2,1:2), PARAMS.alert_limit);
% end
% 
% PHMI= 1 - (1 - PHMI_CA)*PCA0_all*PCA;




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function [IIVN]= associations_tables(z, Nz, Nlm, R, gate)

% Create the nis table
IIVN= ones(Nz,Nlm)*gate;
for i= 1:Nz
    for l= 1:Nlm
        [IIVN(i,l)]= lm_model_and_nis(z(:,i),R,l);        
    end
end
IIVN( IIVN>=gate )= 0;

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function [nis]= lm_model_and_nis(z,R,lm_id)
% return normalised innovation squared (ie, Mahalanobis distance) and normalised distance

global XX PX LM hlm Hlm


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

% calculate H
H = [-xd -yd 0; 
      yd2 -xd2 -1];

% Store the values in the global varibles
hlm{lm_id}= h;
Hlm{lm_id}= H;


% Innovation vector
v= z-h; v(2)= pi_to_pi(v(2));

% Quick check to discard associations
% if abs(v(1)) > 2 || abs(v(2)) > deg2rad(15), nis= inf; return, end;

% Compute IIVN 
S= H*PX*H' + R; 

nis= v'/S*v;
































