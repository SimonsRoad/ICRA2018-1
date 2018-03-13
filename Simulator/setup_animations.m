function h= setup_animations()

global LM PARAMS

% Create lm names
str= {};
for l= 1:size(LM,2)
    str= [str, strcat('lm', num2str(l))];
end

% Create figure
h.figMain=figure('pos',[300 550 1200 500]); hold on; 
xlabel('metres'), ylabel('metres')
set(h.figMain, 'name', 'EKF-localization Simulator')
% axis([-200, 25, -200, 20]);
axis([0, 500, -20, 20]);
axis equal

% Plot LMs
for l= 1:size(LM,2)
    plot(LM(1,l),LM(2,l),'b.','markersize',12);
%     text(LM(1,l)-1,LM(2,l)-2,str(l),'FontSize',5,'color','b');
end

% Initialize dynamic plots
h.xt= patch(0,0,'b'); % vehicle true
h.xv= patch(0,0,'r'); % vehicle estimate
h.xp= patch(0,0,'g'); % vehicle prediction estimate
h.pth= plot(0,0,'k.','markersize',2); % vehicle path estimate
h.obs= plot(0,0,'y'); % observations
h.vcov= plot(0,0,'r'); % vehicle covariance ellipses
h.fcov= plot(0,0,'r'); % feature covariance ellipses
h.path= plot(0,0,'r-'); % path of the vehicle
h.path_true= plot(0,0,'k-'); % True path

% The P(HMI)
h.figHMI= figure('units','centimeters','pos', [0 0 15 10]); hold on; grid on;
set(gca, 'fontsize', 10);
% title('Integrity Risk with Landmark Selection');
h.HMI= plot(0,0,'g-', 'linewidth',2);
h.HMI_CA= plot(0,0,'--b','linewidth',2);
legHMI= legend({'$P(HMI_k)$',...
    '$P(HMI_k | \neg F_K)$'},...
    'Interpreter','latex', 'Location','northwest');
set(legHMI, 'fontsize', 10);
set(gca,'Yscale','log');
xlim([1,PARAMS.numSteps]);
ylim([0,1e-7]);
xlabel('Time epoch')

% Plot the P(CA)_k and P(CA)_K (at current time)
h.figPCA= figure('pos',[2000 0 900 450]); hold on; grid on;
h.PCA_k= plot(0,0,'--b','linewidth',2);
h.PCA_K= plot(0,0,'-g','linewidth',2);
legend({'$P(\neg MA_k | \neg MA_{K-1})$','$P(\neg MA_K)$'}, 'Interpreter','latex');
xlim([1,PARAMS.numSteps]);

% % Plot the cum P(CA) 
% figPCA_sum= figure('pos',[3000 0 900 450]); hold on; grid on;
% title('With Landmark Selection');
% h.PCA_sum= plot(0,0,'-g','linewidth',2);
% legend('P(CA_K)');
% xlim([1,PARAMS.numSteps]);


% Plot the epsilon error and cov envelope online
h.figEps= figure('units','centimeters','pos', [40 0 15 10]); hold on; grid on;
set(gca, 'fontsize', 10);
h.eps= plot(0,0,'-b','linewidth',3);
h.cov= plot(0,0,'--g','linewidth',2);
legEps= legend({'$\hat{\epsilon}_k$','3-$\sigma$ Cov.'},'Interpreter','latex');
set(legEps, 'fontsize', 10);
xlim([1,PARAMS.numSteps]);
xlabel('Time epoch')
ylabel('meters');


% % Plot the XX error and cov envelope online
% figErrorXX= figure('units','centimeters','pos', [60 0 15 10]); hold on; grid on;
% set(gca, 'fontsize', 10);
% h.errorXX= plot(0,0,'-b','linewidth',3);
% h.covXX= plot(0,0,'--g','linewidth',2);
% legErrorXX= legend({'$\hat{\epsilon}_k$ in X-axis','3-$\sigma$ Cov.'},'Interpreter','latex');
% set(legErrorXX, 'fontsize', 10);
% xlim([1,PARAMS.numSteps]);
% xlabel('Time epoch')
% ylabel('meters');

% Plot the IAs online
h.figIA= figure('units','centimeters','pos', [20 0 15 10]); hold on; grid on;
set(gca, 'fontsize', 10);
h.IA= plot(0,0,'-g','linewidth',2);
h.numAssoc= plot(0,0,'--k','linewidth',2);
legIA= legend({'$IA_k$', '# Associations'},'Interpreter','latex');
set(legIA, 'fontsize', 10);
xlim([1,PARAMS.numSteps]);
xlabel('Time epoch')
ylabel('');







