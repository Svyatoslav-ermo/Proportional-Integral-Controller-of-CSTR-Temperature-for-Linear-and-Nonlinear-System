clear all %#ok
close all
clc

%% Q1 For mass and energy balance derivation see report

%% Q2 Steady State

X_ss = zeros(3,4);

X_ss(1,:) = fsolve(@process, [0 0 0 0]);
X_ss(2,:) = fsolve(@process, [0.75 0.75 0.75 0.75]);
X_ss(3,:) = fsolve(@process, [1 1 1 1]);

%% Q3
X = sym('x', [1 4]); syms(X, 'real');

% Create Jacobian
J = sym('dFdx', [4 4]);
F = process(X);

for i = 1:4
    for j = 1:4
        J(i,j) = diff(F(i),X(j));
    end
end

eigenvals = zeros(3,4);
for i = 1:3
    eigenvals(i,:) = eig(double(subs(J, [x1 x2 x3 x4], X_ss(i,:))));
end

%% Q4 For Block Diagram see report

%% Q5 Operating around unsteady steady-state
X = X_ss(2,:);
J = double(subs(J, [x1 x2 x3 x4], X));

%% Q6 Designing PI controller


s = tf('s');
e1 = 12.5;
e2 = 40;
Gp = 8*e1*e2*(s+1.2696)/(s-64.3082)/(s+0.9725)/(s+601.204);
Gd = (s+1.2696)*(s+600)/(s-64.3082)/(s+0.9725)/(s+601.204);

% t_min = 1;
% %  Iterate through different Kc and tauI values
% for Kc = 50:5:600
%     for tauI = 0.001:0.001:0.1
%         Gc = Kc*(1+1/(tauI*s));
%         G = Gp*Gc/(1+Gp*Gc);
%         t = stepinfo(G).SettlingTime;
%         Kc/1000
%         if t < t_min
%             t_min = t;
%             Kc_optimum = Kc;
%             tauI_optimum = tauI;
%         end
%     end
% end
Kc_optimum = 570;
tauI_optimum = 0.013;

Gc = Kc_optimum*(1+1/(tauI_optimum*s));
G = Gp*Gc/(1+Gp*Gc); % Set transfer function

figure(1)
hold on
step(G,0.1)

%% Q7 Applying PI Controller to Linear and Nonlinear System

%  Ysp = 0.005 for Linear system
Ysp = 0.005;
Da2prime = 0;
%  No disturbance
x30 = 0;
Y = Ysp*G;
u = Gc*(Ysp-Y);
t = linspace(0,0.1);
Y_lin = step(Y,t);
u_lin = step(u,t);

%  Ysp = 0.005 for Nonlinear system
[t,Y_nonlin] = ode45(@(t,X_nonlin)DeviationVarProcess(X_nonlin,Ysp,x30,Kc_optimum,tauI_optimum,X,Da2prime),t,zeros(5,1));
%u_nonlin = Kc_optimum*(Ysp - Y_nonlin(:,3) + 1/tauI_optimum*(Ysp - Y_nonlin(:,3)));
u_nonlin = Kc_optimum*(Ysp - Y_nonlin(:,3) + 1/tauI_optimum*Y_nonlin(:,5));

%  Plots for Ysp = 0.005

figure(2)
hold on
sgtitle('Set-Point Change of 0.005')
subplot(2,2,1)
plot(t,u_lin)
title('Manipulated Input Response (u), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,2)
plot(t,Y_lin)
title('Controlled Output Response (Y), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,3);
plot(t,u_nonlin);
title('Manipulated Input Response (u), Nonlinear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,4);
plot(t,Y_nonlin(:,3));
title('Controlled Output Response (Y), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

%  Ysp = 0.01 for Linear system
Ysp = 0.01;
%  No disturbance
x30 = 0;
Y = Ysp*G;
u = Gc*(Ysp-Y);
t = linspace(0,0.1);
Y_lin = step(Y,t);
u_lin = step(u,t);

%  Ysp = 0.01 for Nonlinear system
[t,Y_nonlin] = ode45(@(t,X_nonlin)DeviationVarProcess(X_nonlin,Ysp,x30,Kc_optimum,tauI_optimum,X,Da2prime),t,zeros(5,1));
%u_nonlin = Kc_optimum*(Ysp - Y_nonlin(:,3) + 1/tauI_optimum*(Ysp - Y_nonlin(:,3)));
u_nonlin = Kc_optimum*(Ysp - Y_nonlin(:,3) + 1/tauI_optimum*Y_nonlin(:,5));

figure(3)
hold on
sgtitle('Set-Point Change of 0.01')
subplot(2,2,1)
plot(t,u_lin)
title('Manipulated Input Response (u), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,2)
plot(t,Y_lin)
title('Controlled Output Response (Y), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,3);
plot(t,u_nonlin);
title('Manipulated Input Response (u), Nonlinear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,4);
plot(t,Y_nonlin(:,3));
title('Controlled Output Response (Y), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

%% Q8 Distrubance rejection capabilities of the controller

%  For disturbance of 0.001
x30 = 0.001;
Ysp = 0;
t = linspace(0,0.1);

%  Get step responses for linear system
G = x30*Gd/(1+Gp*Gc); % Transfer function for Ysp = 0
Y_lin = step(G,t);
u_lin = step(Gc,t).*(Ysp-Y_lin);

%  Get step responses for nonlinear system
[t,Y_nonlin] = ode45(@(t,Y_nonlin)DeviationVarProcess(Y_nonlin,Ysp,x30,Kc_optimum,tauI_optimum,X,Da2prime),t,zeros(5,1));
u_nonlin = Kc_optimum*(Ysp - Y_nonlin(:,3) + 1/tauI_optimum*Y_nonlin(:,5));

%  Plot linear and nonlinear responses
figure(4)
hold on
sgtitle('Disturbane of 0.001')
subplot(2,2,1)
plot(t,u_lin)
title('Manipulated Input Response (u), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,2)
plot(t, Y_lin)
title('Controlled Output Response, (Y), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,3)
plot(t,u_nonlin)
title('Manipulated Input Response (u), Nonlinear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,4)
plot(t,Y_nonlin(:,3))
title('Controlled Output Response (Y), Nonlinear')
xlabel('Time (s)')
ylabel('Amplitude')

%  For disturbance of 0.002
%  Linear
x30 = 0.001;
G = x30*Gd/(1+Gp*Gc);
Y_lin = step(G,t);
u_lin = step(Gc,t).*(Ysp-Y_lin);

%  Nonlinear
[t,Y_nonlin] = ode45(@(t,Y_nonlin)DeviationVarProcess(Y_nonlin,Ysp,x30,Kc_optimum,tauI_optimum,X,Da2prime),t,zeros(5,1));
u_nonlin = Kc_optimum*(Ysp - Y_nonlin(:,3) + 1/tauI_optimum*Y_nonlin(:,5));

%  Plot linear and nonlinear responses
figure(5)
hold on
sgtitle('Disturbane of 0.002')
subplot(2,2,1)
plot(t,u_lin)
title('Manipulated Input Response (u), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,2)
plot(t, Y_lin)
title('Controlled Output Response, (Y), Linear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,3)
plot(t,u_nonlin)
title('Manipulated Input Response (u), Nonlinear')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,4)
plot(t,Y_nonlin(:,3))
title('Controlled Output Response (Y), Nonlinear')
xlabel('Time (s)')
ylabel('Amplitude')

%% Q9 Applying PI controller to nonlinear process with Da2prime = 10^5
Da2prime = 10^5;

%  Set point change of 0.005 and no disturbance
Ysp = 0.005;
x30 = 0;
[t,Y_nonlin] = ode45(@(t,Y_nonlin)DeviationVarProcess(Y_nonlin,Ysp,x30,Kc_optimum,tauI_optimum,X,Da2prime),t,zeros(5,1));
u_nonlin = Kc_optimum*(Ysp-Y_nonlin(:,3) + 1/tauI_optimum*Y_nonlin(:,5));

figure(6)
hold on
sgtitle('Nonlinear Responses for Da2'' = 10^5')

subplot(2,2,1)
plot(t,u_nonlin)
title('Manipulated Input Response, Ysp = 0.005')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,2)
plot(t,Y_nonlin(:,3))
title('Controlled Output Response, Ysp = 0.005')
xlabel('Time (s)')
ylabel('Amplitude')

%  Disturbance of 0.001 with no set point change
Ysp = 0;
x30 = 0.001;
[t,Y_nonlin] = ode45(@(t,Y_nonlin)DeviationVarProcess(Y_nonlin,Ysp,x30,Kc_optimum,tauI_optimum,X,Da2prime),t,zeros(5,1));
u_nonlin = Kc_optimum*(Ysp - Y_nonlin(:,3) + 1/tauI_optimum*Y_nonlin(:,5));

subplot(2,2,3)
plot(t,u_nonlin)
title('Manipulated Input Response, Disturbance of 0.001')
xlabel('Time (s)')
ylabel('Amplitude')

subplot(2,2,4)
plot(t,Y_nonlin(:,3))
title('Controlled Output Response, Disturbance of 0.001')
xlabel('Time (s)')
ylabel('Amplitude')

%% Process Function
function F = process(X)

Da1 = 10^6;
Da2 = 10^7;
Da1prime = 1.5*10^6;
Da2prime = 0;
x30 = 0.025;
x40 = 0.025;
E1 = 1.0066;
E2 = 1.0532;
U = 8;
e1 = 12.5;
e2 = 40;
e3 = 1;

F(1) = 1 - X(1) - Da1*exp(-E1/X(3))*X(1);
F(2) = -X(2) + Da1*exp(-E1/X(3))*X(1)-Da2*exp(-E2/X(3))*X(2);
F(3) = x30 - X(3) + Da1prime*exp(-E1/X(3))*X(1) + Da2prime*exp(-E2/X(3))*X(2) - U*(X(3)-X(4));
F(4) = e1*e2*(x40 - X(4)) + U*e1*e3*(X(3) - X(4));

end

function F = DeviationVarProcess(X,Ysp,x30,Kc,tauI,X_ss,Da2prime)

Da1 = 10^6;
Da2 = 10^7;
Da1prime = 1.5*10^6;
E1 = 1.0066;
E2 = 1.0532;
U = 8;
e1 = 12.5;
e2 = 40;
e3 = 1;

F = zeros(4,1);
F(1) = -X(1) - Da1*exp(-E1/(X(3)+X_ss(3)))*(X(1)+X_ss(1)) + Da1*exp(-E1/X_ss(3))*X_ss(1);
F(2) = -X(2) + Da1*exp(-E1/(X(3)+X_ss(3)))*(X(1)+X_ss(1)) - Da1*exp(-E1/X_ss(3))*X_ss(1) - Da2*exp(-E2/(X(3)+X_ss(3)))*(X(2)+X_ss(2)) + Da2*exp(-E2/X_ss(3))*X_ss(2);
F(3) = x30 - X(3) + Da1prime*exp(-E1/(X(3)+X_ss(3)))*(X(1)+X_ss(1)) - Da1prime*exp(-E1/X_ss(3))*X_ss(1) + Da2prime*exp(-E2/(X(3)+X_ss(3)))*(X(2)+X_ss(2)) - Da2prime*exp(-E2/X_ss(3))*X_ss(2) - U*(X(3) - X(4));
F(4) = e1*e2*(Kc*(Ysp - X(3)+1/tauI*X(5)) - X(4)) + U*e1*e3*(X(3)-X(4));
F(5) = Ysp - X(3);

end