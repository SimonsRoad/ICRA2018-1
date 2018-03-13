
function update = integrity_monitoring(step, xx_time, Px_time, xx_update, Px_update, PCA_k)

global XX PX DATA PARAMS SWITCH

% Without associations
P_HMI_CA_time= PHMI_with_CA(xx_time,Px_time(1:2,1:2), PARAMS.alert_limit);
P_HMI_time= 1 + ( P_HMI_CA_time- 1 )* DATA.PCA_K(step-1);

% With associations
P_HMI_CA_update= PHMI_with_CA(xx_update,Px_update(1:2,1:2), PARAMS.alert_limit);
P_HMI_update= 1 + ( P_HMI_CA_update- 1 )* DATA.PCA_K(step-1)* PCA_k;


% Update if necessary
if SWITCH.update_if_necessary && P_HMI_time < P_HMI_update % No update    
    update= 0;
    
    DATA.P_HMI_CA(step)= P_HMI_CA_time;
    DATA.P_HMI(step)= P_HMI_time;
    DATA.PCA_k(step)= 1;
    DATA.PCA_K(step)= DATA.PCA_K(step-1);
else % Update
    update= 1;
    
    XX= xx_update; PX= Px_update;
    
    DATA.P_HMI_CA(step)= P_HMI_CA_update;
    DATA.P_HMI(step)= P_HMI_update;
    DATA.PCA_k(step)= PCA_k;
    DATA.PCA_K(step)= DATA.PCA_K(step-1)*PCA_k;
end
