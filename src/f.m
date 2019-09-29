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

function [X_dot, tau] = f(t,X,type)
t
global my_sigmoid_ a T label a_orign get_qd get_qd_dot lambdac get_qd_dot2 lambda0 lambdaf k0 A Kd P0 K0;

if label=='time_var'
    a=my_sigmoid_(t)*a_orign;
else
    a=a_orign;
end
%Extract elements
qex=X(1:6);      q=qex(3:6);
qex_dot=X(7:12);  q_dot=qex_dot(3:6);

ahat=X(13:16);
i=17;
PInv=[X(i)  X(i+1)  X(i+2)   X(i+3);
   X(i+1)  X(i+4)  X(i+5)   X(i+6);
   X(i+2)  X(i+5)  X(i+7)   X(i+8);
   X(i+3)  X(i+6)  X(i+8)   X(i+9)];
P=PInv^-1;

if t==0
    disp('Checking requirements')
    if (norm(P)>k0)
        error('Required: P(0)<=k0')
    end
end
W=vec2mat(X(27:42),4); %Only the non-diagonal elements are valid here

q_dotf=X(43:46);
tau_filt=X(47:50);

%Compute A and Kd, see paper slotine section 3.2
H_hat_small=diag([ahat(1),ahat(2),ahat(3),ahat(4)]);
Kd=lambdac*H_hat_small;
A=lambdac*eye(4);


%Desired trajectories
qd=get_qd(t);
qd_dot=get_qd_dot(t);
qd_dot2=get_qd_dot2(t);


qtil=q-qd;
qtil_dot=q_dot-qd_dot;

qr_dot=qd_dot - A*qtil;
qr_dot2=qd_dot2 - A*qtil_dot;
s= qtil_dot + A*qtil;

%Compute the diagonal elements of W:
W(1:4+1:end)= lambdaf*(q_dot-q_dotf + [9.81/lambdaf;0;0;0]); %elements that have the joint acceleration.

%Naming convention
phi=q(2);
theta=q(3);
psi=q(4);
phi_dot=q_dot(2);
theta_dot=q_dot(3);
psi_dot=q_dot(4);
m=a(1);
Ixx=a(2);
Iyy=a(3);
Izz=a(4);

%Naming convention (reference)
phir_dot=qr_dot(2);
thetar_dot=qr_dot(3);
psir_dot=qr_dot(4);

zr_dot2=qr_dot2(1);
phir_dot2=qr_dot2(2);
thetar_dot2=qr_dot2(3);
psir_dot2=qr_dot2(4);

Y=[zr_dot2 + 9.81              0                              0                         0                ;
   0                     phir_dot2                -theta_dot*psir_dot         thetar_dot*psi_dot       ;
   0                 phi_dot*psir_dot             thetar_dot2               -phir_dot*psi_dot         ;
   0                 -thetar_dot*phi_dot            theta_dot*phir_dot            psir_dot2           ];

if(type=="PD")
    tau=-20*eye(4)*s;
else
    tau=Y*ahat-Kd*s;
end


%Plant (Dynamic Model of the drone)
C=[0 0 0          0                0                 0            ;
   0 0 0          0                0                 0            ;
   0 0 0          0                0                 0            ;
   0 0 0          0             Izz*psi_dot       -Iyy*theta_dot  ;
   0 0 0     -Izz*psi_dot           0              Ixx*phi_dot    ;
   0 0 0     Iyy*theta_dot     -Ixx*phi_dot              0        ];

B=[1 0 0 0;
   1 0 0 0;
   1 0 0 0;
   0 1 0 0;
   0 0 1 0;
   0 0 0 1];

G=[0 0 a(1)*9.81 0 0 0]';
H=diag([a(1),a(1),a(1),a(2),a(3),a(4)]);
qex_dot2=H\(B*tau-C*qex_dot-G); % second derivative of q_extended
q_dot2=qex_dot2(3:6); 
%%% End of plant

P=PInv^-1;

e=W*ahat-tau_filt;
switch type
    case "PD"
        %Do nothing
        PInv_dot=eye(4); %Don't care about this
    case "TEB"
        P=P0; W=zeros(4); %TEB (tracking error based) controller (FUNCIONA)
        PInv_dot=eye(4); %Don't care about this
    case "CG"
        P=P0; %CG (constant gain) composite adaptive controller (FUNCIONA)
        PInv_dot=eye(4); %Don't care about this
    case "BGF"
        %disp("norm of p=")
        %norm(P)
        PInv_dot=-lambda0*(1-norm(P)/k0)*PInv + W'*W;
    case "CF"
        lambda_t=lambda0; %constant
        PInv_dot=-lambda_t*(PInv - K0^-1) + W'*W;
    otherwise
        disp('Not implemented')
end

ahat_dot=-P*(Y'*s + W'*e);

q_dotf_dot=lambdaf*(q_dot - q_dotf);

%Here we ignore (for now) the diagonal elements of Y1:
Y1=[0              0                                0                         0                 ;
   0                0                          -theta_dot*psi_dot         theta_dot*psi_dot     ;
   0             phi_dot*psi_dot                     0                    -phi_dot*psi_dot      ;
   0             -theta_dot*phi_dot            theta_dot*phi_dot                 0             ];
Wdot=lambdaf*(Y1 - W); %Only the non-diagonal elements are valid here
tmp=Wdot';
Wdot_vector=tmp(:);

tau_filt_dot=lambdaf*(tau-tau_filt);


tmp = PInv_dot';
PInv_dot_vector=tmp(tril(true(size(PInv_dot))));


X_dot = [qex_dot; qex_dot2; ahat_dot; PInv_dot_vector; Wdot_vector; q_dotf_dot; tau_filt_dot];

end