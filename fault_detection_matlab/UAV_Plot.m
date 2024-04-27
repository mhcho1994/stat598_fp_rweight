function UAV_Plot(TIME, Attitude, Altitude, Position, Quadrotor, Fault, Control, Residual, mode, motor_f)

X      =  Attitude{1};
X_R    =  Attitude{2};
Roll   =  Attitude{3};
Pitch  =  Attitude{4};
Yaw    =  Attitude{5};
Xhat   =  Attitude{6};

Z      =  Altitude{1};
Z_R    =  Altitude{2};
Z_D    =  Altitude{3};

XP     =  Position{1};
XP_R   =  Position{2};
X_Posi =  Position{3};
Y_Posi =  Position{4};

X_Position = XP(:,1);
Y_Position = XP(:,2);
Z_Position = Z(:,1);

fault      =  Fault{1};
fault_hat  =  Fault{2};

U1         =  Control{1};
U2         =  Control{2};
U3         =  Control{3};
U4         =  Control{4};

drone1     =  Quadrotor{1};
drone2     =  Quadrotor{2};
drone3     =  Quadrotor{3};
drone4     =  Quadrotor{4};

residual_roll   =  Residual{1}(:,1);
residual_pitch  =  Residual{1}(:,3);
residual_yaw    =  Residual{1}(:,5);

t  = TIME.t;

if mode == 1 
        %% Roll Pitch Yaw Angles (롤 피치 요 각도)
        figure(1)
        set(gcf,'Position',[200, 200, 1100, 500],'Color','w')

        subplot(1,3,1)
        plot(t,X(:,1),t,Roll(:,1),t,Xhat(:,1),'--','LineWidth',2.5);
        xline(4,'green','LineWidth',3)
        legend('$\phi(k)$','$r_{\phi}(k)$','$\hat{\phi}(k)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',14,'fontname','Times New Roman')
        ylabel('Attitude [rad]','fontsize',14,'fontname','Times New Roman')
        title('Roll Angle','fontsize',14,'fontname','Times New Roman')
    
        subplot(1,3,2)
        plot(t,X(:,3),t,Pitch(:,1),t,Xhat(:,3),'--','LineWidth',2.5);
        xline(4,'green','LineWidth',3)
        legend('$\theta(k)$','$r_{\theta}(k)$','$\hat{\theta}(k)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',14,'fontname','Times New Roman')
        ylabel('Attitude [rad]','fontsize',14,'fontname','Times New Roman')
        title('Pitch Angle','fontsize',14,'fontname','Times New Roman')
        
        subplot(1,3,3)
        plot(t,X(:,5),t,Yaw(:,1),t,Xhat(:,5),'--','LineWidth',2.5);
        xline(4,'green','LineWidth',3)
        legend('$\psi(k)$','$r_{\psi}(k)$','$\hat{\psi}(k)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',14,'fontname','Times New Roman')
        ylabel('Attitude [rad]','fontsize',14,'fontname','Times New Roman')
        title('Yaw Angle','fontsize',14,'fontname','Times New Roman')
    
        %% Roll Pitch Yaw Angular Velocity (롤 피치 요 각속도)
        figure(2)
        set(gcf,'Position',[200, 200, 1100, 500],'Color','w')

        subplot(1,3,1)
        plot(t,X(:,2),t,Xhat(:,2),'--','LineWidth',2.5);
        xline(4,'green','LineWidth',3)
        legend('$\dot{\phi}(k)$','$\hat{\dot{\phi}}(k)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',14,'fontname','Times New Roman')
        ylabel('Attitude [rad/s]','fontsize',14,'fontname','Times New Roman')
        title('Roll Angular Velocity','fontsize',14,'fontname','Times New Roman')
    
        subplot(1,3,2)
        plot(t,X(:,4),t,Xhat(:,4),'--','LineWidth',2.5);
        xline(4,'green','LineWidth',3)
        legend('$\dot{\theta}(k)$','$\hat{\dot{\theta}}(k)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',14,'fontname','Times New Roman')
        ylabel('Attitude [rad/s]','fontsize',14,'fontname','Times New Roman')
        title('Pitch Angular Velocity','fontsize',14,'fontname','Times New Roman')
    
        subplot(1,3,3)
        plot(t,X(:,6),t,Xhat(:,6),'--','LineWidth',2.5);
        xline(4,'green','LineWidth',3)
        legend('$\dot{\psi}(k)$','$\hat{\dot{\psi}}(k)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',14,'fontname','Times New Roman')
        ylabel('Attitude [rad/s]','fontsize',14,'fontname','Times New Roman')
        title('Yaw Angular Velocity','fontsize',14,'fontname','Times New Roman')

        %% Position (위치)
        figure(3) 
        set(gcf,'Position',[200, 200, 1100, 500],'Color','w')

        subplot(1,3,1)
        plot(t,XP(:,1),t,X_Posi(:,1),'--','LineWidth',2.5);
        legend('$x(k)$','$r_{x}(k)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',14,'fontname','Times New Roman')
        ylabel('Position [m]','fontsize',14,'fontname','Times New Roman')
        title('X Position','fontsize',14,'fontname','Times New Roman')
        
        subplot(1,3,2)
        plot(t,XP(:,2),t,Y_Posi(:,1),'--','LineWidth',2.5);
        legend('$y(k)$','$r_{y}(k)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',14,'fontname','Times New Roman')
        ylabel('Position [m]','fontsize',14,'fontname','Times New Roman')
        title('Y Position','fontsize',14,'fontname','Times New Roman')
    
        subplot(1,3,3)
        plot(t,Z(:,1),t,Z_D(:,1),'--','LineWidth',2.5);
        legend('$z(k)$','$r_{z}(k)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',14,'fontname','Times New Roman')
        ylabel('Altitude [m]','fontsize',14,'fontname','Times New Roman')
        title('Z Position','fontsize',14,'fontname','Times New Roman')

        % %% Faults (폴트)
        % figure(4)
        % set(gcf,'Position',[200, 200, 1100, 500],'Color','w')
        % 
        % subplot(1,3,1)
        % plot(t,fault(:,1),t,fault_hat(:,1),'--','LineWidth',2.5);
        % legend('$u_{2f}(t)$','$\hat{u}_{2f}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        % grid on;
        % xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        % ylabel('Roll [N-M]','fontsize',12,'fontname','Times New Roman')
        % 
        % subplot(1,3,2)
        % plot(t,fault(:,2),t,fault_hat(:,2),'--','LineWidth',2.5);
        % legend('$u_{3f}(t)$','$\hat{u}_{3f}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        % grid on;
        % xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        % ylabel('Pitch [N-M]','fontsize',12,'fontname','Times New Roman')
        % 
        % subplot(1,3,3)
        % plot(t,fault(:,3),t,fault_hat(:,3),'--','LineWidth',2.5);
        % legend('$u_{4f}(t)$','$\hat{u}_{4f}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        % grid on;
        % xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        % ylabel('Yaw [N-M]','fontsize',12,'fontname','Times New Roman')

        %% Control Input
        % figure(5)
        % 
        % subplot(2,2,1)
        % plot(t,U1(:,1),'LineWidth',2.5);
        % legend('$u_{T}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        % grid on;
        % xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        % ylabel('Total Thrust','fontsize',12,'fontname','Times New Roman')
        % 
        % subplot(2,2,2)
        % plot(t,U2(:,1),'LineWidth',2.5);
        % legend('$u_{\phi}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        % grid on;
        % xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        % %ylabel('Roll Torque','fontsize',12,'fontname','Times New Roman')
        % 
        % subplot(2,2,3)
        % plot(t,U3(:,1),'LineWidth',2.5);
        % legend('$u_{\theta}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        % grid on;
        % xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        % %ylabel('Pitch Torque','fontsize',12,'fontname','Times New Roman')
        % 
        % subplot(2,2,4)
        % plot(t,U4(:,1),'LineWidth',2.5);
        % legend('$u_{\psi}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        % grid on;
        % xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        % %ylabel('Yaw Thrust','fontsize',12,'fontname','Times New Roman')
        
        %% Residual
        figure(6)
        set(gcf,'Position',[200, 200, 1100, 500],'Color','w')
        
        subplot(1,3,1)
        plot(t,residual_roll,'LineWidth',2.5);
        legend('$e_{\phi}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        ylabel('Roll Angle Residual','fontsize',12,'fontname','Times New Roman')
      
        subplot(1,3,2)
        plot(t,residual_pitch,'LineWidth',2.5);
        legend('$e_{\theta}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        ylabel('Pitch Angle Residual','fontsize',12,'fontname','Times New Roman')

        subplot(1,3,3)
        plot(t,residual_yaw,'LineWidth',2.5);
        legend('$e_{\psi}(t)$','interpreter','latex','FontSize',15,'FontWeight','bold');
        grid on;
        xlabel('Time [s]','fontsize',12,'fontname','Times New Roman')
        ylabel('Yaw Angle Residual','fontsize',12,'fontname','Times New Roman')

else
    %% Video Plots
    writerObj = VideoWriter('Quadrotor UAV Tracking Control','MPEG-4');
    open(writerObj);

    x_init = 0;
    y_init = 0;
    z_init = 0;

    AXIS   = 3;

    figure(1)
    set(gcf,'Position',[200, 200, 900, 600],'Color','w')

    for j = 1:50:30000
        
        if (0<=j) && (j<=4000)
        else
            TMP1  = 'Motor Fault';
            TMP2  = ': ';
            TMP3  = num2str(motor_f);
            TMP4  = strcat(TMP1,TMP2,TMP3);
            F     = annotation('textbox',[0.122 0.765 0.3 0.2],'String',TMP4,'Color','red','Fontsize',15,'EdgeColor','none');
        end

        plot3(X_Posi(:,1),Y_Posi(:,1),Z_D(:,1),'-.','LineWidth',3); hold on
        plot3(XP(:,1),XP(:,2),Z(:,1),'LineWidth',2); hold on
        xlabel('X Position [m]','fontsize',15,'fontname','Times New Roman')
        ylabel('Y Position [m]','fontsize',15,'fontname','Times New Roman')
        zlabel('Z Position [m]','fontsize',15,'fontname','Times New Roman')

        % Center
        scatter3(X_Position(j), Y_Position(j), Z_Position(j), 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
        axis([-AXIS, x_init+AXIS, -AXIS, y_init+AXIS, 0, z_init+AXIS]); grid on;

        % [1]
        scatter3(drone1(1,j), drone1(2,j), drone1(3,j), 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
        plot3([X_Position(j) drone1(1,j)], [Y_Position(j) drone1(2,j)], [Z_Position(j) drone1(3,j)],'LineWidth', 1.5, 'Color', 'red'); hold on
        textscatter3(real([drone1(1,j) drone1(2,j) 0.2+drone1(3,j)]), string(1), 'MarkerSize',12);

        % [4]
        scatter3(drone2(1,j), drone2(2,j), drone2(3,j), 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
        plot3([X_Position(j) drone2(1,j)], [Y_Position(j) drone2(2,j)], [Z_Position(j) drone2(3,j)],'LineWidth', 1.5, 'Color', 'blue'); hold on
        textscatter3(real([drone2(1,j) drone2(2,j) 0.2+drone2(3,j)]), string(4), 'MarkerSize',12);

        % [3]
        scatter3(drone3(1,j), drone3(2,j), drone3(3,j),'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
        plot3([X_Position(j) drone3(1,j)], [Y_Position(j) drone3(2,j)], [Z_Position(j) drone3(3,j)],'LineWidth', 1.5, 'Color', 'red'); hold on
        textscatter3(real([drone3(1,j) drone3(2,j) 0.2+drone3(3,j)]), string(3), 'MarkerSize',12);

        % [2]
        scatter3(drone4(1,j), drone4(2,j), drone4(3,j), 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
        plot3([X_Position(j) drone4(1,j)], [Y_Position(j) drone4(2,j)], [Z_Position(j) drone4(3,j)],'LineWidth', 1.5, 'Color', 'blue'); hold on
        textscatter3(real([drone4(1,j) drone4(2,j) 0.2+drone4(3,j)]), string(2), 'MarkerSize',12);

        hold on
        axis([-AXIS, x_init+AXIS, -AXIS, y_init+AXIS, 0, z_init+AXIS]);
        grid on; 
        hold off

        vertex    = [1 1 2 ; 1 1 2; 1 1 2; 1 1 2; 1 1 2 ; 1 1 2; 1 1 2; 1 1 2]; 
        face      = [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8]; 
        patch('Vertices',vertex,'faces',face,'Facecolor',[1 1 0]);

        view(3)

        j
        
        %% Retrieve the current frame of the figure
        F = getframe(figure(1));

        %% Write the current frame to the writer object
        writeVideo(writerObj,F)

    end

    %% Close the writer object. File appears in the current folder!
    close(writerObj);
    disp("Video File Written Successfully!")

end

end