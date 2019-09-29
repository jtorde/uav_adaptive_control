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
function [t,y]=adapt_control(type)
global tt get_qd get_qd_dot get_qd_dot2 input_pd input_teb input_cg input_bgf input_cf T get_qd1 get_qd2 get_qd3 get_qd4;
init_vector=[zeros(1,12),zeros(1,4),10000*[1,0,0,0,1,0,0,1,0,1],[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],zeros(1,4),zeros(1,4)];
interval=[0 T];

%Symbolic expressions
qd1=get_qd1_sym(tt);  qd2=get_qd2_sym(tt);  qd3=get_qd2_sym(tt);  qd4=get_qd2_sym(tt);

qd1_dot=get_qd1_dot_sym(tt);
qd2_dot=get_qd2_dot_sym(tt);

qd1_dot2=get_qd1_dot2_sym(tt);
qd2_dot2=get_qd2_dot2_sym(tt);

qd_symbolic=[qd1; qd2; qd3; qd4];
qd_dot_symbolic=[qd1_dot; qd2_dot; qd2_dot; qd2_dot];
qd_dot2_symbolic=[qd1_dot2; qd2_dot2; qd2_dot2; qd2_dot2];

get_qd = matlabFunction(qd_symbolic);
get_qd_dot= matlabFunction(qd_dot_symbolic);
get_qd_dot2 = matlabFunction(qd_dot2_symbolic);

get_qd1=matlabFunction(qd1);
get_qd2=matlabFunction(qd2);
get_qd3=matlabFunction(qd3);
get_qd4=matlabFunction(qd4);

[t,y] = ode45(@(t,y) f(t,y,type),interval,init_vector);
%And now let's compute tau:
for i=1:size(t,1)
    [X_dot, tau] = f(t(i),y(i,:)',type);
    
    switch type
        case "PD"
            input_pd=[input_pd; [tau']];
        case "TEB"
            input_teb=[input_teb; [tau']];
        case "CG"
            input_cg=[input_cg; [tau']];
        case "BGF"
            input_bgf=[input_bgf; [tau']];
        case "CF"
            input_cf=[input_cf; [tau']];
        otherwise
            disp('Not implemented')
    end
end


end

function ret = get_qd1_sym(tt)
amp1=2;
ret=amp1*(1-cos(pi*tt));
end

function ret = get_qd2_sym(tt)
amp2=pi/4;
ret=amp2*(1-cos(pi*tt));
end


function ret = get_qd1_dot_sym(tt)
ret=diff(get_qd1_sym(tt),tt);
end

function ret = get_qd2_dot_sym(tt)
ret=diff(get_qd2_sym(tt),tt);
end

function ret = get_qd1_dot2_sym(tt)
ret=diff(diff(get_qd1_sym(tt),tt),tt);

end

function ret = get_qd2_dot2_sym(tt)
ret=diff(diff(get_qd2_sym(tt),tt),tt);
end