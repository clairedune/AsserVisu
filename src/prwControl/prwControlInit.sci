// Copyright (C) INRIA 1999-2009
// 
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License version 2 as published
// by the Free Software Foundation.
// 
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.


setActuationSamplingPeriod(2.5e-3);
setActuationSamplingDelay(0.1e-3);
setNbActuators(30);


printf('\nStarting Trajectory Generation...\n');
tic();
//[Q, QDOT, QDDOT, ZMP, Support] = WalkingTrajectory_withFP(%F);
printf('    Elapsed time: %g seconds\n', toc());

//pause
Visu(Q)
//pause
COP = zeros(2, size(Q, 2));
COPBounds = zeros(4, size(Q, 2));
for k = 1:size(Q, 2),
  D = Inertia(Q(:, k))*QDDOT(:, k)+NLEffects(Q(:, k), QDOT(:, k));
  C = ContactJacobian(Q(:, k));
  cop = Contact(Q(:, k));
  pause
  if Support(k) == 1 then contact = [%T; %T; %T; %T; %F; %F; %F; %F];
  elseif Support(k) == -1 then contact = [%F; %F; %F; %F; %T; %T; %T; %T];
  else contact = [%T; %T; %T; %T; %T; %T; %T; %T];
  end;
  C = C([contact; contact; contact], :);
  lambda = pinv(C(:, $-5:$)')*D($-5:$);
  cop = [cop(1:$/3)'; cop(2*$/3+1:$)'];
  cop = cop(:, contact);
  COPBounds(:, k) = [min(cop, 'c'); max(cop, 'c')];
  COP(:, k) = sum(cop*diag(lambda($/3+1:2*$/3)), 'c')/sum(lambda($/3+1:2*$/3));
end;

xset('window', 20);
subplot(1,2,1);
plot([COP(1, :)', COPBounds([1, 3], :)', ZMP(1, :)']);

subplot(1,2,2)
xname('COP X/Z (HuMAnS)');
plot([COP(2, :)', COPBounds([2, 4], :)', ZMP(2, :)']);Tend/ActuationSamplingPeriod

CreateVRML(1e-2, Q(:, 1:2:$), getHAnimJointsNames(), 'HRP2', 'High', 'HRP2WalkModular.wrl', 1, zeros(Q(1:24, 1:2:$)), getHAnimContactSolidsNames(), getHAnimContactSolidsRGB());
Visu(Q, lambda, POV=[77, 77]);
