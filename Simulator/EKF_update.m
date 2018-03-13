function [xx,Px]= EKF_update(xx,Px,z,idf)

global PARAMS h hlm H Hlm

Nz= size(z,2);

% make R a block diagonal with higher dimension
R= kron(eye(nnz(idf)),PARAMS.R);

% create the models for the association
h= []; H= [];
for i= 1:Nz
    if idf(i) ~= 0
        H= [H; Hlm{idf(i)}];
        h= [h; hlm{idf(i)}];
    else
        z(1,i)= NaN;
    end
end

% Eliminate non-associated msmts
z( :, isnan(z(1,:)) )= [];

% If at least one msmts is associated
if ~isempty(z)
    % Compute innovations
    gamma= z(:) - h;
    gamma(2:2:end)= pi_to_pi(gamma(2:2:end));
    Y= H*Px*H' + R;
    % ngamma= gamma'*(Y\gamma);
    
    % Update the estimate
    K= Px*H'/Y;
    Px= Px - K*H*Px;
    xx= xx + K*gamma;
end