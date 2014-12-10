span=100;
[legPos1,freq1,gyro1,Torque1,t1,Robot1]=getLegMovement('leftOnly.txt',span);
[legPos2,freq2,gyro2,Torque2,t2,Robot2]=getLegMovement('rightOnly.txt',span);
[legPos3,freq3,gyro3,Torque3,t3,Robot3]=getLegMovement('leftOnly2.txt',span);
[legPos4,freq4,gyro4,Torque4,t4,Robot4]=getLegMovement('rightOnly2.txt',span);

subplot = @(m,n,p) subtightplot (m, n, p, [0.07 0.07], [0.1 0.07], [0.1 0.08]);

legPosL=0.5*(legPos1(:,2)+legPos3(1:end-2,2));
legPosR=0.5*(legPos2(:,1)+legPos4(1:end-2,1));
freqL=0.5*(freq1(:,4)+freq3(1:end-2,4));
freqR=0.5*(freq2(:,3)+freq4(1:end-2,3));

DCR=Robot2.DCR*4096;
DCL=Robot1.DCL*4096;

figure(1)
subplot(2,1,1)
plot(t2,legPosR,t1,legPosL)
title('Leg position (radians)')
ylabel('leg position')
legend('right','left','Location','NorthWest')
subplot(2,1,2)
[haxes,hline1,hline2]=plotyy([t2(1:end-1) t1(1:end-1)],[freqR freqL],t2,DCR)
title('Leg frequency (Hz)')
set(haxes(2),'ytick',0:500:4000)



