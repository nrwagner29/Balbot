addpath "C:\Users\neilrw2\Box\Balbot\Balbot_Data_Storage\PiDatalogging"

clear
clc
close all

A = readmatrix("trial_3--24-9-5.csv");

q = [A(:,2) A(:,3) A(:,4) A(:,5) A(:,6) A(:,7) A(:,8) A(:,9) A(:,10) A(:,11)];

second = 0;
for i=1:length(A(:,1))-1
    %CCODE from  PI

    % Tauleft(i) = .1*sin(A(i,1)/1000000 * 6.28);
    % Tauright(i) = .1*cos(A(i,1)/1000000 * 6.28);
    % Tauleft(i) =  .1*A(i,3);
    % Tauright(i) = .1*A(i,4);
    kp = 1;
    kd = .1;
    offleft = 0;
    doffleft = 0;
    Tauleft(i) = -kp*(offleft - A(i,3)) + -kd * (doffleft - A(i,10));

    offright = 0;
    doffright = 0;
    Tauright(i) = -kp*(offright - A(i,4)) + -kd * (doffright - A(i,9));
    kt = 0.053;                % torque constant
    leftcurr = Tauleft(i) / kt;   %// amps
    rightcurr = Tauright(i) / kt; %// amps
    %/*current limiter*/
    currlimit = 5; %// AMPS
    if (leftcurr > currlimit)

        leftcurr = currlimit;
    end
    if (leftcurr < -currlimit)

        leftcurr = -currlimit;
    end
    if (rightcurr > currlimit)

        rightcurr = currlimit;
    end
    if (rightcurr < -currlimit)

        rightcurr = -currlimit;
    end


    eddq4(i) = kt*leftcurr/0.000539;
    eddq5(i) = kt*rightcurr/0.000539; %assume no viscous damping


    A(i,1) = A(i,1) + second * 1000000;
    if (A(i,1) > A(i+1,1))
        second = second + 1;
    end
end


A(end,1) = A(end,1) + second * 1000000;
t = A(:,1)/1000000000;
eddq4 = eddq4';
eddq5 = eddq5';


hold on
set(gca,'ColorOrderIndex',4)
plot(t,q(:,1),t,q(:,2),t,q(:,3),'-','LineWidth',1);
hold off
ylabel('rad')
xlabel('Time [sec]')
grid on
title('states (q)')
legend('q1','q2','q3')

figure
hold on
set(gca,'ColorOrderIndex',4)
plot(t,q(:,6),t,q(:,7),t,q(:,8),'-','LineWidth',1);
hold off
ylabel('rad/s')
xlabel('Time [sec]')
grid on
title('states (dq)')
legend('dq1','dq2','dq3')

figure
hold on
set(gca,'ColorOrderIndex',4)
plot(t,q(:,4),t,q(:,5),'-','LineWidth',1);
hold off
ylabel('rad')
xlabel('Time [sec]')
grid on
title('wheel states (q)')
legend('q4','q5')

figure
hold on
set(gca,'ColorOrderIndex',4)
plot(t,q(:,9),t,q(:,10),t(1:end-1),eddq4(:)',t(1:end-1),eddq5(:)','-','LineWidth',1);
hold off
ylabel('rad/s')
xlabel('Time [sec]')
grid on
title('actual and expected wheel states (dq)')
legend('dq4','dq5','exp ddq4','exp ddq5')