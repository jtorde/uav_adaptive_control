% /****************************************************************************
%  *   Copyright (c) 2019 Jesus Tordesillas Torres. All rights reserved.
%  *
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions
%  * are met:
%  *
%  * 1. Redistributions of source code must retain the above copyright
%  *    notice, this list of conditions and the following disclaimer.
%  * 2. Redistributions in binary form must reproduce the above copyright
%  *    notice, this list of conditions and the following disclaimer in
%  *    the documentation and/or other materials provided with the
%  *    distribution.
%  * 3. Neither the name of this repo nor the names of its contributors may be
%  *    used to endorse or promote products derived from this software
%  *    without specific prior written permission.
%  *
%  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
%  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
%  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
%  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
%  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
%  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
%  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
%  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
%  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  * POSSIBILITY OF SUCH DAMAGE.
%  *
%  ****************************************************************************/
clc;close all;clear all;
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

set(groot, 'DefaultAxesFontWeight', 'bold');
%set(0,'DefaultFigureWindowStyle','docked')
set(0,'DefaultFigureWindowStyle','normal')
global my_sigmoid_ a a_orign label torque_vector amp1 amp2 tt input_pd input_teb input_cg input_bgf input_cf lambda0 lambdaf lambdac T get_qd1 get_qd2 get_qd3 get_qd4 k0 A Kd P0 K0; 

my_sigmoid_ = @my_sigmoid;

label='1'; %1 or time_var. If time_var, the parameters will change as a sigmoid
print_plots=0;

if strcmp(label,'time_var')
    T=3;
else
    T=1.5;
end

m=0.5;
Ixx=2*4.8e-02;
Iyy=2*4.8e-02;
Izz=2*8.8e-02;

a=[m;Ixx;Iyy;Izz]; %Real parameters
a_orign=[m;Ixx;Iyy;Izz]; %Real parameters


%Parameters only of CF:
K0=50*eye(4); 

%Parameters of CF and BGF:
lambda0=4800; 
k0=200;   %

%Parameters of CG and BGF and CF:
lambdaf=10; 

%Parameters of TEB and CG:
P0=5*eye(4); 

%Parameters of all the algorithms (used for the control law and the definition of s):
lambdac=5; 

% A=20*eye(4);
% Kd=50*eye(4);

thrust.teb=[];     thrust.cg=[];   thrust.bgf=[];
tau_yaw.teb=[];   tau_yaw.cg=[];  tau_yaw.bgf=[];
tau_pitch.teb=[];  tau_pitch.cg=[];  tau_pitch.bgf=[];
tau_roll.teb=[];   tau_roll.cg=[];  tau_roll.bgf=[];

tau4_vector=[];
syms tt

%opts = odeset('RelTol',1e-2);
   % [t,Y] = ode45(@(t,y) myfunc(t,y,....), tspan, ic, opts);

[t_pd,y_pd] = adapt_control("PD"); %"TEB", "CG" or "BGF" 
[t_teb,y_teb] = adapt_control("TEB"); %"TEB", "CG" or "BGF"
[t_cg,y_cg] = adapt_control("CG"); %"TEB", "CG" or "BGF"
[t_bgf,y_bgf] = adapt_control("BGF"); %"TEB", "CG" or "BGF"
[t_cf,y_cf] = adapt_control("CF"); %"TEB", "CG" or "BGF" 

%% Plots
close all
set(gcf,'renderer','Painters')
close all

width=1500; height=300;

%%%%%% Estimated Parameters
init_par=13;
figure; 
set(gcf, 'Position',  [100, 100, width, height])
subplot(1,4,1); hold on
plot(t_teb,y_teb(:,init_par))
plot(t_cg,y_cg(:,init_par))
plot(t_bgf,y_bgf(:,init_par))
plot(t_cf,y_cf(:,init_par))
if strcmp(label,'time_var')
    plot(t_cf,my_sigmoid_(t_cf)*a_orign(1),'Color','blue','LineStyle','--')
else
    line([0 T],[a_orign(1) a_orign(1)],'Color','blue','LineStyle','--');
end
%yline(a(1),'-.b')
title('$\hat{m}\;\;(kg)$'); xlabel('$t(s)$')
legend('TEB','CG','BGF','CF','Real','Location','southeast')

subplot(1,4,2); hold on
plot(t_teb,y_teb(:,init_par+1))
plot(t_cg,y_cg(:,init_par+1))
plot(t_bgf,y_bgf(:,init_par+1))
plot(t_cf,y_cf(:,init_par+1))
if strcmp(label,'time_var')
    plot(t_cf,my_sigmoid_(t_cf)*a_orign(2),'Color','blue','LineStyle','--')
else
    line([0 T],[a_orign(2) a_orign(2)],'Color','blue','LineStyle','--');
end
title('$\hat{I}_{xx}\;\;(kg \; m^2)$'); xlabel('$t(s)$')
legend('TEB','CG','BGF','CF','Real','Location','southeast')

subplot(1,4,3); hold on
plot(t_teb,y_teb(:,init_par+2))
plot(t_cg,y_cg(:,init_par+2))
plot(t_bgf,y_bgf(:,init_par+2))
plot(t_cf,y_cf(:,init_par+2))
if strcmp(label,'time_var')
    plot(t_cf,my_sigmoid_(t_cf)*a_orign(3),'Color','blue','LineStyle','--')
else
    line([0 T],[a_orign(3) a_orign(3)],'Color','blue','LineStyle','--');
end
title('$\hat{I}_{yy}\;\;(kg \; m^2)$'); xlabel('$t(s)$')
legend('TEB','CG','BGF','CF','Real','Location','southeast')

subplot(1,4,4); hold on
plot(t_teb,y_teb(:,init_par+3))
plot(t_cg,y_cg(:,init_par+3))
plot(t_bgf,y_bgf(:,init_par+3))
plot(t_cf,y_cf(:,init_par+3))
if strcmp(label,'time_var')
    plot(t_cf,my_sigmoid_(t_cf)*a_orign(4),'Color','blue','LineStyle','--')
else
    line([0 T],[a_orign(4) a_orign(4)],'Color','blue','LineStyle','--');
end

title('$\hat{I}_{zz}\;\;(kg \; m^2)$'); xlabel('$t(s)$')
legend('TEB','CG','BGF','CF','Real','Location','southeast')

set(gcf,'renderer','Painters')
%suptitle('Estimated Parameters')
if(print_plots==1)
 printeps(1,strcat('./figs/estimated_parameters',label))
end

%%%%%% Errors in q
figure
init_q=3;
subplot(1,4,1); hold on
plot(t_teb,y_teb(:,init_q)-get_qd1(t_teb))
plot(t_cg,(y_cg(:,init_q)-get_qd1(t_cg)))
plot(t_bgf,(y_bgf(:,init_q)-get_qd1(t_bgf)))
plot(t_cf,(y_cf(:,init_q)-get_qd1(t_cf)))
plot(t_pd,y_pd(:,init_q)-get_qd1(t_pd))
title('$\tilde{z}$ (m)'); xlabel('$t(s)$')
legend({'TEB','CG','BGF','CF','PD'},'Location','southeast')
%ylim([-0.45 0.1])

subplot(1,4,2); hold on
plot(t_teb,(180/pi)*(y_teb(:,init_q+1)-get_qd2(t_teb)))
plot(t_cg,(180/pi)*(y_cg(:,init_q+1)-get_qd2(t_cg)))
plot(t_bgf,(180/pi)*(y_bgf(:,init_q+1)-get_qd2(t_bgf)))
plot(t_cf,(180/pi)*(y_cf(:,init_q+1)-get_qd2(t_cf)))
plot(t_pd,(180/pi)*(y_pd(:,init_q+1)-get_qd2(t_pd)))
title('$\widetilde{roll}$ (deg)'); xlabel('$t(s)$')
legend('TEB','CG','BGF','CF','PD','Location','southeast')
%ylim([-2 1])

subplot(1,4,3); hold on
plot(t_teb,(180/pi)*(y_teb(:,init_q+2)-get_qd3(t_teb)))
plot(t_cg,(180/pi)*(y_cg(:,init_q+2)-get_qd3(t_cg)))
plot(t_bgf,(180/pi)*(y_bgf(:,init_q+2)-get_qd3(t_bgf)))
plot(t_cf,(180/pi)*(y_cf(:,init_q+2)-get_qd3(t_cf)))
plot(t_pd,(180/pi)*(y_pd(:,init_q+2)-get_qd3(t_pd)))
title('$\widetilde{pitch}$ (deg)'); xlabel('$t(s)$')
legend('TEB','CG','BGF','CF','PD','Location','southeast')
%ylim([-2 1])

subplot(1,4,4); hold on
plot(t_teb,(180/pi)*(y_teb(:,init_q+3)-get_qd4(t_teb)))
plot(t_cg,(180/pi)*(y_cg(:,init_q+3)-get_qd4(t_cg)))
plot(t_bgf,(180/pi)*(y_bgf(:,init_q+3)-get_qd4(t_bgf)))
plot(t_cf,(180/pi)*(y_cf(:,init_q+3)-get_qd4(t_cf)))
plot(t_pd,(180/pi)*(y_pd(:,init_q+3)-get_qd4(t_pd)))
title('$\widetilde{yaw} (deg)$'); xlabel('$t(s)$')
legend('TEB','CG','BGF','CF','PD','Location','southeast')
%ylim([-2 1])

%suptitle('Position Errors')
set(gcf, 'Position',  [100, 100, width, height])
set(gcf,'renderer','Painters')
if(print_plots==1)
 printeps(2,strcat('./figs/position_errors',label))
end

%%%%%%%%%%%%%%%%%   INPUTS
figure;  
set(gcf, 'Position',  [100, 100, width, height])
subplot(1,4,1); hold on
plot(t_teb,input_teb(:,1))
plot(t_cg,input_cg(:,1))
plot(t_bgf,input_bgf(:,1))
plot(t_cf,input_cf(:,1))
plot(t_pd,input_pd(:,1))
%yline(a(1),'-.b')
title('$T\;\;(N)$'); xlabel('$t(s)$'); xlim([0 T])
legend('TEB','CG','BGF','CF','PD','Location','southeast')

subplot(1,4,2); hold on
plot(t_teb,input_teb(:,2))
plot(t_cg,input_cg(:,2))
plot(t_bgf,input_bgf(:,2))
plot(t_cf,input_cf(:,2))
plot(t_pd,input_pd(:,2))
%yline(a(1),'-.b')
title('$\tau_{roll}\;\;(Nm)$'); xlabel('$t(s)$'); xlim([0 T])
legend('TEB','CG','BGF','CF','PD','Location','southeast')
subplot(1,4,3); hold on
plot(t_teb,input_teb(:,3))
plot(t_cg,input_cg(:,3))
plot(t_bgf,input_bgf(:,3))
plot(t_cf,input_cf(:,3))
plot(t_pd,input_pd(:,3))
%yline(a(1),'-.b')
title('$\tau_{pitch}\;\;(Nm)$'); xlabel('$t(s)$'); xlim([0 T])
legend('TEB','CG','BGF','CF','PD','Location','southeast')

subplot(1,4,4); hold on
plot(t_teb,input_teb(:,4))
plot(t_cg,input_cg(:,4))
plot(t_bgf,input_bgf(:,4))
plot(t_cf,input_cf(:,4))
plot(t_pd,input_pd(:,4))
%yline(a(1),'-.b')
title('$\tau_{yaw}\;\;(Nm)$'); xlabel('$t(s)$'); xlim([0 T])
legend('TEB','CG','BGF','CF','PD','Location','southeast')

%suptitle('Inputs')
set(gcf,'renderer','Painters')
if(print_plots==1)
 printeps(3,strcat('./figs/inputs',label))
end

%%%%%%%%%%%%%%%%%%% TRAJECTORIES
% figure
% subplot(3,1,1); hold on
% plot(t_teb,get_qd1(t_teb));
% plot(t_teb,y_teb(:,init_q));
% legend('z_d (m)','z (m)')
% 
% subplot(3,1,2)
% hold on
% plot(t_teb,(180/pi)*get_qd2(t_teb));
% plot(t_teb,(180/pi)*get_qd3(t_teb));
% plot(t_teb,(180/pi)*get_qd4(t_teb));
% legend('roll_d (deg)','pitch_d (deg)','yaw_d(deg)')
% 
% subplot(3,1,3); hold on
% plot3(y_teb(:,1),y_teb(:,2),y_teb(:,3));
% plot3(y_cg(:,1),y_cg(:,2),y_cg(:,3));
% plot3(y_bgf(:,1),y_bgf(:,2),y_bgf(:,3));
% title('3D trajectory'); xlabel('x'); ylabel('y'); zlabel('z'); grid on
% legend('TEB','CG','BGF')
%%

function y=my_sigmoid(t)
   global T
   y=(1+0.5*sigmoid(t,T/2,80));
end

function [yaw pitch roll]=obtainEulerFromAccelAndYaw(accel,yaw) 
    %See Mellinger paper (Minimum Snap Trajectory Generation...)
    tmp=[accel(1);accel(2);accel(3) + 9.81];
    zb=tmp/norm(tmp);
    xc=[cos(yaw);sin(yaw);0];
    yb=cross(zb,xc)/norm(cross(zb,xc));
    xb=cross(yb,zb);
    rot_matrix = [xb,yb,zb];
    [yaw pitch roll] = rotm2eul(rot_matrix,'ZYX');
end






