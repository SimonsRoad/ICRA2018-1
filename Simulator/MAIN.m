
dbstop if error
dbclear if error


clear; close all;
configfile; % ** USE THIS FILE TO CONFIGURE THE EKF-SLAM **

h= setup_animations();


% *****************    MAIN LOOP    *****************
for step= 2:PARAMS.numSteps
    disp(['Step: ',num2str(step)]);
    
    % Compute controls
    [G,iwp]= compute_steering(xtrue, iwp, G);
    
    % EKF predict step
    [xtrue,XX,PX]= predict (xtrue,XX,PX,G);
    
    % Integrity prediction
    [xPred,ensureTime]= integrity_prediction(XX,PX,G,iwp,DATA.PCA_K(step-1)); 
    
    % Integrity monitoring
    PCA_k= prob_nMA(XX,PX);

    % Get measurements
    [z,idft]= get_observations(xtrue);
    z= add_observation_noise(z);
    
    % DA
    if  ~isempty(z)        
        if SWITCH.association == 0
            [gamma,H,Y,R,DATA.PCA(step), DATA.PCA_MJ(step)]= data_associate_known(z,idft);
        elseif SWITCH.association == 1
            [idf]= data_associate_LNN(z);
        end
                
        % Update
        [xx_update,Px_update]= EKF_update(XX,PX,z, idf);
        
        % Integrity monitoring
        [update]= integrity_monitoring(step, XX, PX, xx_update, Px_update, PCA_k);
        
        % store msmts data
        if update
            DATA.IA(step)= any( (idft - idf).*idf );
            DATA.numAssoc(step)= nnz( idf );
        end
    else
        update= 0;
        
        DATA.PCA_k(step)= 1;
        DATA.PCA_K(step)= DATA.PCA_K(step-1);
        DATA.P_HMI_CA(step)= PHMI_with_CA(XX,PX(1:2,1:2), PARAMS.alert_limit);
        DATA.P_HMI(step)= 1 + ( DATA.P_HMI_CA(step)- 1 )* DATA.PCA_K(step-1);
    end
    
    % Store DATA
    store_data(step, xtrue, update);        
    
    % Save video
    if step > 1 && step < 950
        % Plots
        do_plots(xtrue, xPred, z, h, step);
%         set(0, 'CurrentFigure', h.figMain)
%         set(gca,'nextplot','replacechildren');
% %         set(h.figMain,'Renderer','zbuffer');
% 
%         DATA.frames(step-810)= getframe(h.figMain);
    end

    %     pause(0.5);
end 
% *****************  END OF MAIN LOOP    *****************

% post_processing_and_plots(step)



