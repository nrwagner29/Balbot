addpath "C:\Users\neilrw2\Box\Balbot\Balbot_Data_Storage\PiDatalogging\"

clear
clc
close all

A = readmatrix("trial_4--24-8-30.csv");

q = [A(:,2) A(:,3) A(:,4) A(:,5) A(:,6) A(:,7) A(:,8) A(:,9) A(:,10) A(:,11)];

second = 0;
for i=1:length(A(:,1))-1
    
A(i,1) = A(i,1) + second * 1000000;
    if (A(i,1) > A(i+1,1))
        second = second + 1;
    end
end
A(end,1) = A(end,1) + second * 1000000;
t = A(:,1)/1000000;

hold on
set(gca,'ColorOrderIndex',4)
plot(t,q(:,1),t,q(:,2),t,q(:,3),'-','LineWidth',1.5);
hold off
ylabel('rad')
xlabel('Time [sec]')
grid on
title('states (q)')
legend('q1','q2','q3')

figure
hold on
set(gca,'ColorOrderIndex',4)
plot(t,q(:,6),t,q(:,7),t,q(:,8),'-','LineWidth',1.5);
hold off
ylabel('rad/s')
xlabel('Time [sec]')
grid on
title('states (dq)')
legend('dq1','dq2','dq3')

figure
hold on
set(gca,'ColorOrderIndex',4)
plot(t,q(:,4),t,q(:,5),t,q(:,9),t,q(:,10),'-','LineWidth',1.5);
hold off
ylabel('rad/s and rad')
xlabel('Time [sec]')
grid on
title('wheel states (dq and q)')
legend('q4','q5','dq4','dq5')