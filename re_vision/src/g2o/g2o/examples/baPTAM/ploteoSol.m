clear all
close all
run g2o


figure; axis equal; hold on;
TcwGT1 = reshape(camsGT(1,3:end),4,[]);
TwcGT1 = inv(TcwGT1);
TcwGT2 = reshape(camsGT(2,3:end),4,[]);
TwcGT2 = inv(TcwGT2);

TcGT1cGT2 = TcwGT1 * TwcGT2;

TcwF1 = reshape(camsF(1,3:end),4,[]);
TwcF1 = inv(TcwF1);
TcwF2 = reshape(camsF(2,3:end),4,[]);
TwcF2 = inv(TcwF2);
TcF1cF2 = TcwF1 * TwcF2;

escala = norm(TcGT1cGT2(1:3,4))/norm(TcF1cF2(1:3,4))
escala = 1
%escala = norm(TwcGT2(1:3,4) - TwcGT1(1:3,4)) / norm(TwcF2(1:3,4) - TwcF1(1:3,4));

for c = 1: size(camsGT,1)
Tcw = reshape(camsGT(c,3:end),4,[]);    
Twc = inv(Tcw);
plot3(Twc(1,4), Twc(2,4), Twc(3,4), '*b')

Tcw = reshape(cams0(c,3:end),4,[]);    
Twc = inv(Tcw);
plot3(Twc(1,4), Twc(2,4), Twc(3,4), 'dk')

Tcw = reshape(camsF(c,3:end),4,[]);    
Twc = inv(Tcw);
plot3(escala*Twc(1,4), escala*Twc(2,4), escala*Twc(3,4), 'og')
end

plot3(ptosGT(:,2), ptosGT(:,3), ptosGT(:,4),'.k');
plot3(ptos0(:,2), ptos0(:,3), ptos0(:,4),'xr');
plot3(escala*ptosF(:,2), escala*ptosF(:,3), escala*ptosF(:,4),'og')


figure; axis equal; hold on;
% plot3(ptosGT(:,2), ptosGT(:,3), ptosGT(:,4),'.k');
% plot3(ptos0(:,2), ptos0(:,3), ptos0(:,4),'xr');
% plot3(ptosF(:,2), ptosF(:,3), ptosF(:,4),'og')

for c = 1: size(camsGT,1)
Tcw = reshape(camsGT(c,3:end),4,[]);    
Twc = inv(Tcw);
plot3(Twc(1,4), Twc(2,4), Twc(3,4), '*b')

Tcw = reshape(cams0(c,3:end),4,[]);    
Twc = inv(Tcw);
plot3(Twc(1,4), Twc(2,4), Twc(3,4), 'dk')

Tcw = reshape(camsF(c,3:end),4,[])   
Twc = inv(Tcw);
plot3(escala*Twc(1,4), escala*Twc(2,4), escala*Twc(3,4), 'og')
end

kGT
k0
kF