function store_data(step, xtrue, update)

global DATA XX PX

alpha= [-sin(XX(3)); cos(XX(3)); 0];

% Store the results

DATA.errorXX(step,:)= abs(xtrue - XX)';
DATA.stdXX(step,:)= 3*sqrt(diag(PX)');

DATA.eps(step)= (alpha'* (xtrue - XX) );
DATA.stdEps(step)= 3*sqrt(alpha'*PX*alpha);

DATA.path(step,:)= XX(1:2)';

% 
% if ~update 
%     DATA.PCA_k(step)= 1;
%     DATA.PCA_K(step)= DATA.PCA_K(step-1);
%     if isempty(idft)
%         DATA.P_HMI_CA(step)= PHMI_with_CA(XX, PX(1:2,1:2), PARAMS.alert_limit);
%         DATA.P_HMI(step)= 1 + ( P_HMI_CA- 1 )* DATA.PCA_K(step-1);
%     else
%         DATA.P_HMI_CA(step)= P_HMI_CA;
%         DATA.P_HMI(step)= P_HMI;
%     end
% end


