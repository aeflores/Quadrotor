syms dt omega p q r q0 q1 q2 q3
qk0 = [q0; q1; q2; q3];
OMEGA = [ 0 -p -q -r;
          p  0  r  q;
          q -r  0  p;
          r  q -p  0]/2;

qk1 = (eye(4)*cos(dt*omega/2) + sin(dt*omega/2)/omega*OMEGA)*qk0

yaw = 0.0;
pitch =  -7.84;
pitch =  6;
roll =  0.0;	
radtodeg = 180/pi;

q = angle2quat(0 , pitch/radtodeg, roll/radtodeg)
qoffset = quatinv(q)




% // Q3 = Q2 * Q1
% // Qsi= Qsf* Qfi
% // Qsf⁻¹ * Qsi = Qfi
% // qoffset = Qsf⁻¹
% // qout = qoffset * qsi
  


