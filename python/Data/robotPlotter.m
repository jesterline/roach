clear all

Robot = pullRobot('ground_torqueVsPhase_tripod.txt'); %--> suspended 
%run, felt beats in string  and altered DC until they disappeared; L,R=1900,2374 
t = Robot.t;
legPosRight = Robot.legPosRight;
legPosLeft = Robot.legPosLeft;
legComRight = Robot.legComRight;
legComLeft = Robot.legComLeft;
DCR = Robot.DCR;
DCL = Robot.DCL;
BEMFR = Robot.BEMFR;
BEMFL = Robot.BEMFL;
vBatt = Robot.vBatt;


figure(1)
subplot(2,1,1)
plot(t,DCR,t,DCL)
subplot(2,1,2)
plot(t,BEMFR,t,BEMFL)
% tStep=t(2)-t(1);
% freqRight=diff(legPosRight)/tStep/(2*pi);
% freqLeft=diff(legPosLeft)/tStep/(2*pi);
% span = 150;
% freqRightMA=movingAve(freqRight,span);
% freqLeftMA=movingAve(freqLeft,span);
% [haxes,hline1,hline2]=plotyy(t(1:end-1),[freqRightMA freqLeftMA],t,DCR*4096)
% set(haxes(2),'ylim',[0.25*4096 4096],'ytick',0:100:4096,'xlim',[0 max(t)])
% set(haxes(1),'ylim',[0 25],'ytick',0:3:30,'xlim',[0 max(t)])

ttotal=ceil(length(t)/1000);
tStep=t(2)-t(1);
freqRight=diff(legPosRight)/tStep/(2*pi);
freqLeft=diff(legPosLeft)/tStep/(2*pi);
legPosRight=legPosRight-min(legPosRight);
legPosLeft=legPosLeft-min(legPosLeft);
figure(2)
subplot(2,1,1)
plot(t,legPosRight,t,legPosLeft)
legend('Right','Left')
xlabel('Time (s)')
ylabel('Leg position (radians)')
set(gca,'xtick',0:2:ttotal)
subplot(2,1,2)
legPosDiff=legPosRight-legPosLeft;
plot(t,legPosDiff/max(abs(legPosDiff)),t,movingAve(mod(legPosDiff/2/pi,1),1))
%plot(t,movingAve(mod(legPosDiff/2/pi,1),100))
ylabel({'Left/right side leg position difference','(cycles)'})
% mean(legPosDiff(9000:17000))

span = 150;
freqRightMA=movingAve(freqRight,span);
freqLeftMA=movingAve(freqLeft,span);
figure(3)
subplot(2,1,1)
plot(t(1:end-1),freqRightMA,'k',t(1:end-1),freqLeftMA,'r')
legend('Right side','Left side')
xlabel('Time (s)')
ylabel('Leg frequency (Hz)')
set(gca,'xtick',0:2:ttotal)
hold on

gyroX=Robot.gyroX;
gyroY=Robot.gyroY;
gyroZ=Robot.gyroZ;
subplot(2,1,2)
freqDiff=movingAve(freqRightMA-freqLeftMA,200);
plot(t(1:end-1),freqDiff)
% legend('Right','Left')
ylabel('Leg frequency difference (Hz)')
% xlabel('DC (%)')

figure(4)
plot(t,gyroX,t,gyroY,t,gyroZ)
legend('x','y','z')
ylabel('Gyro (rad/s)')
set(gca,'xtick',0:2:ttotal)

% xlX=Robot.xlX;
% xlY=Robot.xlY;
% xlZ=Robot.xlZ;
% subplot(3,1,3)
% plot(t,xlX,t,xlY,t,xlZ)
% legend('x','y','z')
% ylabel('Acceleration (m/s^2)')

 freqLeftAve=mean(freqLeft(1000:end))
 freqRightAve=mean(freqRight(1000:end))
 
 %%
 %nice figures
 
 [legPos2,freq2,gyro2,torque2,t2,Robot2]=getLegMovement('Rjump2200.txt',span);

figure(1)
ha = tight_subplot(2,1,[.01 .03],[.1 .03],[.12 .08]);
set(ha,'FontName','CMU Serif');
legPosDiff2=(legPos2(:,1)-legPos2(:,2))/2/pi;
DCR=Robot2.DCR*100;
DCL=Robot2.DCL*100;
axes(ha(1))
[haxes,hline1,hline2]=plotyy(t2(1:end-1),[freq2(:,3) freq2(:,4)],t2,[DCR DCL]);
set(hline2(1),'color',[.6 .1 .1])
set(hline2(2),'color','b')
set(haxes(2),'ycolor','k','box','off')
set(haxes(1),'box','off')
hl=legend('$\dot{\theta}_{R}$','$\dot{\theta}_{L}$','Right duty cycle','Left duty cycle','Location','NorthWest');
set(hl,'Interpreter','latex','fontsize',11)
ylabel(haxes(1),'Leg frequency (Hz)')
ylabel(haxes(2),'Duty cycle (%)')
linkaxes(haxes,'x')
axis([0 max(t2) min(min(freq2(:,3:4)))-2 max(max(freq2(:,3:4)))+2])
set(haxes(1),'xlim',[0 max(t2)],'ylim',[1 11])
set(haxes(2),'ylim',[45 90])
set(haxes(1),'ytick',0:2:24)
set(haxes(2),'ytick',0:10:90)
set(haxes,'xticklabel',[])
set(hline2,'linewidth',2)
set(hline1,'linewidth',2)
set(hline1(1),'color','r')
set(hline1(2),'color',[0 0.6 0.95])

axes(ha(2))
plot(t2,legPosDiff2,'k','linewidth',2);
mean(legPosDiff2(8000:20000))
xlabel('Time (s)')
ylabel({'Difference in leg phase,','\theta_{L}-\theta_{R} (cycles)'})
axis([0 max(t2) min(legPosDiff2)-5 max(legPosDiff2)+5])

export_fig jumps -pdf -transparent

%%
%nice gyro
span=100;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('Rjump2200.txt',span);

figure(6)
ha = tight_subplot(2,1,[0.01 .03],[.09 .01],[.1 .02]);
set(ha,'FontName','CMU Serif');

axes(ha(1))
torqueR=torque1(:,1);
torqueL=torque1(:,2);
tqs=plot(t1,torqueR/1000,'k',t1,torqueL/1000,'r:','linewidth',2);
axis([min(t1) max(t1) 0.14 0.27])
set(ha(1),'xticklabel',[],'ytick',0.15:.03:0.26)
% fill([8 18 18 8],[-20 -20 30 30],[.9 .9 .9],'edgecolor','none')
% hold on
axes(ha(2))
hgy=plot(t1,gyro1(:,1),t1,gyro1(:,2),'--',t1,gyro1(:,3),':','linesmoothing','on');
xlabel('Time (s)')
ylabel({'Angular velocity','(rad/s)'})
legend([hgy(1) hgy(2) hgy(3)],{'Roll','Pitch','Yaw'},'location','southeast')
axis([0 max(t1) -24 23])
% str(1)={'Well-synchronized'};
% str(2)={'region'};
% tx=text(13,-10,str,'horizontalalignment','center','fontsize',22);
% set(tx,'backgroundcolor',[.95 .95 .95],'edgecolor',[.5 .5 .5],'fontname','CMU Serif')
set(ha,'layer','top')
set(hgy,'linewidth',2)

export_fig jumpsGyro -pdf -transparent

%%
%torque & junk
span=100;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('RControl2300.txt',span);
DCR=Robot1.DCR;
DCL=Robot1.DCL;

figure(4)
ha = tight_subplot(3,1,[0.01 .03],[.1 .07],[.14 .27]);
set(ha,'FontName','CMU Serif');

axes(ha(1))
[haxes,hline1,hline2]=plotyy(t1(1:end-1),[freq1(:,3) freq1(:,4)],t1,[DCR*100 DCL*100]);
set(hline2(1),'color',[.6 .1 .1])
set(hline2(2),'color','b')
set(haxes(2),'ycolor','k','box','off')
set(haxes(1),'box','off')
%hl=legend('$\dot{\theta}_{R}$','$\dot{\theta}_{L}$','Right duty cycle','Left duty cycle','Location','NorthWest');
%set(hl,'Interpreter','latex','fontsize',11)
ylabel(haxes(1),'Leg frequency (Hz)')
ylabel(haxes(2),'Duty cycle (%)')
linkaxes(haxes,'x')
axis([0 max(t1) min(min(freq1(:,3:4)))-2 max(max(freq1(:,3:4)))+2])
set(haxes(1),'xlim',[0 max(t1)],'ylim',[-2 14])
set(haxes(2),'ylim',[-5 145])
set(haxes(1),'ytick',0:2:24)
set(haxes(2),'ytick',0:25:100)
set(haxes,'xticklabel',[])
set(hline2,'linewidth',2)
set(hline1,'linewidth',2)
set(hline1(1),'color','r')
set(hline1(2),'color',[0 0.6 0.95])

axes(ha(2))
torqueR=movingAve(torque1(:,1),1);%./DCR;
torqueL=movingAve(torque1(:,2),1);%./DCL;
% mean(torqueR(10:1000))
% mean(torqueR(10000:15000))
% mean(torqueL(10:10000))
% mean(torqueL(10000:15000))
tqs=plot(t1,torqueR/1000,t1,torqueL/1000,'k');
axis([0 max(t1) -0.05 0.4])
ylabel({'Motor torque','(N\cdotm)'})
set(ha(2),'xticklabel',[])
set(tqs(1),'color',[.1 .6 .1])

axes(ha(3))
% plot(t1,legPos1(:,1),t1,legPos1(:,2))
% legPosDiff=(legPos1(:,1)-legPos1(:,2))/2/pi;
% mean(legPosDiff(1000:9000))
% poss=plot(t1(1:15000),legPos1(1:15000,:)/2/pi)
% xlabel('Time (s)')
% ylabel({'Leg phase, \theta_{R}-\theta_{L}','(cycles)'})
% axis([0 max(t1(1:15000)) min(legPos1(1:15000,1))/2/pi-5 max(legPos1(1:15000,1))/2/pi+5])
% set(ha(3),'ytick',0:40:240)
gy=plot(t1,gyro1);
axis([min(t1) max(t1) -18 20])
xlabel('Time (s)')
ylabel({'Angular velocity','(rad/s)'})

hl=legend([hline1(1) hline1(2) hline2(1) hline2(2) tqs(1) tqs(2) gy(1) gy(2) gy(3)],...
    '$\dot{\theta_{R}}$','$\dot{\theta_{L}}$','$DC_R$','$DC_L$',...
    '$\tau_{R}$','$\tau_{L}$','Roll','Pitch','Yaw','Location','NorthEastOutside');
set(hl,'Interpreter','latex')
legp=get(hl,'position');
set(hl,'position',[.75 .2 legp(3:4)])

export_fig Rcontrol2300 -pdf -transparent
%%

%zoom in above plot

figure(5)
ha = tight_subplot(3,1,[0.01 .03],[.06 .05],[.06 .05]);
set(ha,'FontName','CMU Serif');
rangeL=14100;
rangeU=14800;

axes(ha(1))
DCR=Robot1.DCR*100;
DCL=Robot1.DCL*100;
DCdiff=(DCR-DCL)./DCL*100;
freqR=freq1(:,3);
freqL=freq1(:,4);
haxes=plot(t1(rangeL:rangeU),[freqR(rangeL:rangeU) freqL(rangeL:rangeU)],'linewidth',2);

%axis([0 max(t1(1:15000)) min(min([freqR(1:15000) freqL(1:15000)]))-2 max(max([freqR(1:15000) freqL(1:15000)]))+2])
%set(haxes,'xlim',[min(t1(rangeL:rangeU)) max(t1(rangeL:rangeU))])
xlim([min(t1(rangeL:rangeU)) max(t1(rangeL:rangeU))]);
%set(haxes,'ylim',[10 16])
ylim([6.5 9.5]);
set(haxes(1),'color','r')
set(haxes(2),'color',[0 0.6 0.95])

axes(ha(2))
torqueR=torque1(:,1);
torqueL=torque1(:,2);
tqs=plot(t1(rangeL:rangeU),torqueR(rangeL:rangeU)/1000,'k',t1(rangeL:rangeU),torqueL(rangeL:rangeU)/1000,'k');
axis([min(t1(rangeL:rangeU)) max(t1(rangeL:rangeU)) 0.13 0.21])
set(ha(2),'xticklabel',[],'ytick',0.02:.03:0.27)
set(tqs(1),'color',[.1 .6 .1])

axes(ha(3))
% plot(t1,legPos1(:,1),t1,legPos1(:,2))
% legPosDiff=(legPos1(:,1)-legPos1(:,2))/2/pi;
% poss=plot(t1(rangeL:rangeU),legPos1(rangeL:rangeU,:)/2/pi);
% axis([min(t1(rangeL:rangeU)) max(t1(rangeL:rangeU)) min(legPos1(rangeL:rangeU,1))/2/pi-.5 max(legPos1(rangeL:rangeU,2))/2/pi+.5])
plot(t1(rangeL:rangeU),gyro1(rangeL:rangeU,:))
axis([min(t1(rangeL:rangeU)) max(t1(rangeL:rangeU)) -12 12])

% hl=legend([hline1(1) hline1(2) hline2 tqs(1) tqs(2) poss(1) poss(2)],...
%     '$\dot{\theta_{R}}$','$\dot{\theta_{L}}$','$DC_R$-$DC_L$',...
%     '$\tau_{R}$','$\tau_{L}$','$\theta_{R}$','$\theta_{L}$','Location','NorthEastOutside');
% set(hl,'Interpreter','latex')
% legp=get(hl,'position');
% set(hl,'position',[.75 .2 legp(3:4)])
 
%%
span=100;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('PWMsync3000.txt',span);
DCR=Robot1.DCR*100;
DCL=Robot1.DCL*100;

ttotal=ceil(length(t1)/1000);
tStep=t(2)-t(1);
freqRight=diff(legPos1(:,1))/tStep/(2*pi);
freqLeft=diff(legPos1(:,2))/tStep/(2*pi);
freqRightMA=movingAve(freqRight,span);
freqLeftMA=movingAve(freqLeft,span);
legPosRight=legPos1(:,1)-min(legPos1(:,1));
legPosLeft=legPos1(:,2)-min(legPos1(:,2));
figure(2)
subplot(3,1,1)
%[haxes,hline1,hline2]=plotyy(t1(1:end-1),[freqRightMA freqLeftMA],t1,DCR);
plot(t1,DCL,'m',t1,DCR,'g','linewidth',2)
ylim([0 100])
xlim([0 max(t1)])
%set(haxes(2),'ylim',[56 85],'ytick',0:5:100,'ycolor','k','xticklabel',[],'xlim',[0 max(t1)])
%set(haxes(1),'ylim',[9 19],'ytick',0:3:30,'xlim',[0 max(t1)],'box','off')
ylabel({'Right side', 'duty cycle (%)'})
%legend('Right freq','Left freq','Right DC','location','southeast')

subplot(3,1,2)
%freqDiffMAEx=movingAve(freqLeft-freqRight,900);
plot([0 15],[0 0],'k:',t1(1:end-1),freqLeftMA-freqRightMA,'b','linewidth',2);
%hold on
%plot(t1(1:end-1),freqDiffMAEx,'r','linewidth',3);
xlim([0 max(t1)])
ylim([-3 10])
ylabel({'Difference in','leg frequency (Hz)'})
%hold off

subplot(3,1,3)
legPosDiff=legPosLeft-legPosRight;
plot(t1,mod(legPosDiff/2/pi,1),'r')
xlim([0 max(t1)])
xlabel('Time (s)')
ylabel({'Difference in', 'leg position','(portion of cycle)'})

%%
%roll bar under robot, comparing gyro
span=50;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('sus2_bar_none.txt',span);
%[legPos2,freq2,gyro2,torque2,t2,Robot2]=getLegMovement('sus2_bar_close.txt',span);
[legPos3,freq3,gyro3,torque3,t3,Robot3]=getLegMovement('sus2_bar_mid.txt',span);
%[legPos4,freq4,gyro4,torque4,t4,Robot4]=getLegMovement('sus_bar_far.txt',span);

figure(1)
subplot(3,1,1)
plot(t1,gyro1(:,1),'k',t3,gyro3(:,1),'g')%,t4,gyro4(:,1))
title('Roll')
ylabel('rad/s')
legend('normal','weighted')
subplot(3,1,2)
plot(t1,gyro1(:,2),'k',t3,gyro3(:,2),'g')%,t4,gyro4(:,2))
ylabel('rad/s')
title('Pitch')
subplot(3,1,3)
plot(t1,gyro1(:,3),'k',t3,gyro3(:,3),'g')%,t4,gyro4(:,3))
ylabel('rad/s')
xlabel('Time (s)')
title('Yaw')

figure(2)
subplot(3,1,1)
plot(t1,Robot1.DCR*100)
axis([0 t1(end) 0 100])
ylabel('Duty cycle (%)')
subplot(3,1,2)
plot(t1(1:end-1),freq1(:,3),'k',t1(1:end-1),freq1(:,4),'r')
legend('Right','Left')
axis([0 t1(end) 10 23])
ylabel({'Unaltered robot','Leg frequency (Hz)'})
subplot(3,1,3)
plot(t3(1:end-1),freq3(:,3),'k',t3(1:end-1),freq3(:,4),'r')
axis([0 t1(end) 10 23])
ylabel({'Weighted robot','Leg frequency (Hz)'})
xlabel('Time (s)')

%%
%3D gyro plot -- checking loopiness
span=50;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('sus_bar_none.txt',span);

figure(1)
subplot(3,1,1)
plot(gyro1(12000:13000,1),gyro1(12000:13000,2))
title('Roll/Pitch')
subplot(3,1,2)
plot(gyro1(12000:13000,1),gyro1(12000:13000,3))
title('Roll/Yaw')
subplot(3,1,3)
plot(gyro1(12000:13000,2),gyro1(12000:13000,3))
title('Pitch/Yaw')

figure(2)
plot3(gyro1(6000:10000,1),gyro1(6000:10000,2),gyro1(6000:10000,3))
xlabel('Roll (rad/s)')
ylabel('Pitch (rad/s)')
zlabel('Yaw (rad/s)')
grid on

figure(3)
plot3(gyro1(22000:26000,1),gyro1(22000:26000,2),gyro1(22000:26000,3))
xlabel('Roll (rad/s)')
ylabel('Pitch (rad/s)')
zlabel('Yaw (rad/s)')
grid on
%%
%lab meeting 2/20/15
clear all
span=50;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('sus_Rwave_1850_3150.txt',span);
[legPos2,freq2,gyro2,torque2,t2,Robot2]=getLegMovement('ground_Rwave_2350_3650.txt',span);
DCR1=Robot1.DCR*100;
DCL1=Robot1.DCL*100;
DCR2=Robot2.DCR*100;
DCL2=Robot2.DCL*100;
legPosDiff1=legPos1(:,1)-legPos1(:,2);
legPosDiff2=legPos2(:,1)-legPos2(:,2);


figure(1)
subplot(3,1,1)
plot(t1,[DCR1 DCL1]);
axis([0 t1(end) 0 100])
ylabel('Duty Cycle (%)')
legend('Right','Left')
subplot(3,1,2)
plot(t1(1:end-1),freq1(:,3),t1(1:end-1),freq1(:,4))
axis([0 t1(end) 12 23])
ylabel('Leg frequency (Hz)')
subplot(3,1,3)
plot(t1,mod(legPosDiff1/2/pi,1),'r')
axis([0 t1(end) 0 1])
ylabel({'Difference in leg phase' ,'(fraction of cycle)'})
xlabel('Time (s)')

figure(2)
subplot(3,1,1)
plot(t2,[DCR2 DCL2]);
axis([0 t2(end) 0 100])
ylabel('Duty Cycle (%)')
legend('Right','Left')
subplot(3,1,2)
plot(t2(1:end-1),freq2(:,3),t2(1:end-1),freq2(:,4))
axis([0 t2(end) 5 24])
ylabel('Leg frequency (Hz)')
subplot(3,1,3)
plot(t2,mod(legPosDiff2/2/pi,1),'r')
axis([0 t2(end) 0 1])
ylabel({'Difference in leg phase' ,'(fraction of cycle)'})
xlabel('Time (s)')

%%
%exploring torque vs. phase
span=50;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('sus_Rwave_1850_3150.txt',span);
legPhase=mod(legPos1,2*pi);
DCR=Robot1.DCR;
DCL=Robot1.DCL;

figure(1)
a=subplot(1,2,1);
plot(torque1(2000:3000,1)./DCR(2000:3000),legPhase(2000:3000,1),'.',torque1(2000:3000,2)./DCL(2000:3000),legPhase(2000:3000,2))
ylabel('Phase (rad)')
xlabel('Torque')
axis([200 400 0 2*pi])
legend('Right','Left','location','southeast')
text(290,6,'Left side leads')
b=subplot(1,2,2);
plot(torque1(24000:25000,1)./DCR(24000:25000),legPhase(24000:25000,1),torque1(24000:25000,2)./DCL(24000:25000),legPhase(24000:25000,2))
%plot(torque1(13000:14000,1)./DCR(13000:14000),legPhase(13000:14000,1),torque1(13000:14000,2)./DCL(13000:14000),legPhase(13000:14000,2))
xlabel('Torque')
axis([200 400 0 2*pi])
text(290,6,'Left side leads')
set(gcf,'NextPlot','add');
axes;
h = title('Not synchronized');
set(gca,'Visible','off');
set(h,'Visible','on');

figure(2)
subplot(1,2,1)
plot(torque1(6000:7000,1)./DCR(6000:7000),legPhase(6000:7000,1),torque1(6000:7000,2)./DCL(6000:7000),legPhase(6000:7000,2),':')
ylabel('Phase (rad)')
xlabel('Torque')
axis([200 400 0 2*pi])
legend('Right','Left','location','southeast')
subplot(1,2,2)
%plot(torque1(15000:16000,1)./DCR(15000:16000),legPhase(15000:16000,1),torque1(15000:16000,2)./DCL(15000:16000),legPhase(15000:16000,2))
plot(torque1(11000:12000,1)./DCR(11000:12000),legPhase(11000:12000,1),torque1(11000:12000,2)./DCL(11000:12000),legPhase(11000:12000,2))
xlabel('Torque')
axis([200 400 0 2*pi])
set(gcf,'NextPlot','add');
axes;
h = title('Synchronized');
set(gca,'Visible','off');
set(h,'Visible','on');

%ground running
span=50;
[legPos2,freq2,gyro2,torque2,t2,Robot2]=getLegMovement('ground_Rwave_2350_3650.txt',span);
legPhase=mod(legPos2,2*pi);
DCR=Robot2.DCR;
DCL=Robot2.DCL;

figure(3)
a=subplot(1,2,1);
%DCR=3050, DCL=2350
plot(torque2(26000:27000,1)./DCR(26000:27000),legPhase(26000:27000,1),torque2(26000:27000,2)./DCL(26000:27000),legPhase(26000:27000,2))
ylabel('Phase (rad)')
xlabel('Torque')
axis([200 400 0 2*pi])
legend('Right','Left','location','southeast')
text(290,6,'Right side leads')
b=subplot(1,2,2);
%DCR=3050, DCL=3650
plot(torque2(13500:14500,1)./DCR(13500:14500),legPhase(13500:14500,1),torque2(13500:14500,2)./DCL(13500:14500),legPhase(13500:14500,2))
%plot(torque1(13000:14000,1)./DCR(13000:14000),legPhase(13000:14000,1),torque1(13000:14000,2)./DCL(13000:14000),legPhase(13000:14000,2))
xlabel('Torque')
axis([200 400 0 2*pi])
text(290,6,'Left side leads')
set(gcf,'NextPlot','add');
axes;
h = title('Not synchronized');
set(gca,'Visible','off');
set(h,'Visible','on');

figure(4)
subplot(1,2,1)
%DCR=3050, DCL=3050
%plot(torque2(7200:8200,1)./DCR(7200:8200),legPhase(7200:8200,1),torque2(7200:8200,2)./DCL(7200:8200),legPhase(7200:8200,2))
plot(torque2(7200:8200,1),legPhase(7200:8200,1),torque2(7200:8200,2),legPhase(7200:8200,2))
ylabel('Phase (rad)')
xlabel('Torque')
axis([50 400 0 2*pi])
legend('Right','Left','location','southeast')
subplot(1,2,2)
%DCR=3050, DCL=2650
%plot(torque1(15000:16000,1)./DCR(15000:16000),legPhase(15000:16000,1),torque1(15000:16000,2)./DCL(15000:16000),legPhase(15000:16000,2))
%plot(torque2(22500:23500,1)./DCR(22500:23500),legPhase(22500:23500,1),torque2(22500:23500,2)./DCL(22500:23500),legPhase(22500:23500,2))
plot(torque2(22500:23500,1),legPhase(22500:23500,1),torque2(22500:23500,2),legPhase(22500:23500,2))
xlabel('Torque')
axis([50 400 0 2*pi])
set(gcf,'NextPlot','add');
axes;
h = title('Synchronized');
set(gca,'Visible','off');
set(h,'Visible','on');

%%
%examining torque patterns in static walking
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('sus_torqueVsPhase_tripod.txt',span);
[legPos2,freq2,gyro2,torque2,t2,Robot2]=getLegMovement('ground_torqueVsPhase_tripod.txt',span);

legPhase1=mod(legPos1,2*pi);
legPhase2=mod(legPos2,2*pi);
figure(1)

%torque from crank
subplot(3,1,1)
plot(legPhase1(:,1),torque1(:,1),'.',legPhase1(:,2),torque1(:,2),'.',[0 10],[0 0],'k-')
title('Crank/Flexures')
axis([0 2*pi -100 200])
ylabel('Torque')

%torque from ground+crank
subplot(3,1,2)
plot(legPhase2(:,1),torque2(:,1),'.',legPhase2(:,2),torque2(:,2),'.',[0 10],[0 0],'k-')
title('Ground + Crank/Flexures')
axis([0 2*pi -100 200])
ylabel('Torque')

%torque from ground alone
legPhase3=(legPhase1+legPhase2)/2;
torque3=torque2-torque1;
subplot(3,1,3)
plot(legPhase3(:,1),torque3(:,1),'.',legPhase3(:,2),torque3(:,2),'.',[0 10],[0 0],'k-')
title('Ground Alone')
axis([0 2*pi -100 200])
ylabel('Torque')
xlabel('Leg phase (rad)')