z=1100;
velo=[V(z,1);V(z,2)]*0.001;
Fee=[Fe(z,1);Fe(z,2)];
dE=E(z)*0.001/0.001;
A=[velo',0,0;0,0,velo'];
B=[velo(1)^2;velo(1)*velo(2);velo(1)*velo(2);velo(2)^2];
cin=A'*Fee;
C=[cin;-dE];
D=[-A'*A,-B;B',0];
uu=0.0000000000001;
un=uu*eye(5,5);
%Xm=pinv(D)*C;
Xm=inv(D'*D+un)*D'*C;
Alm=[Xm(1:2)';Xm(3:4)']
%inv(hh)*D'*C;
(Alm*velo)'*velo
Alm*velo