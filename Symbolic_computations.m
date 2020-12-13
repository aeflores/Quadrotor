syms deltaomega p q r q0 q1 q2 q3
qk0 = [q0; q1; q2; q3];
OMEGA = [ 0 -p -q -r;
          p  0  r  q;
          q -r  0  p;
          r  q -p  0]/2;

qk1 = (eye(4)*cos(deltaomega/2) + sin(deltaomega/2)*deltaomega*OMEGA)*qk0


