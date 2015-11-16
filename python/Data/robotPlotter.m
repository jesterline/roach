clear all

span=50;
[legPos1,freq1,gyro1,torque1,t1,Robot]=getLegMovement('new_sus_data.txt',span);
[legPos2,freq2,gyro2,torque2,t2,Robot2]=getLegMovement('Mismatched_noSprings2.txt',span);
[legPos3,freq3,gyro3,torque3,t3,Robot3]=getLegMovement('acrylic_1R_12L.txt',span);

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
ylabel('Duty cycle')
subplot(2,1,2)
plot(t,BEMFR,t,BEMFL)
ylabel('BEMF')
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
legPosRight=legPosRight;
legPosLeft=legPosLeft;

figure(2)
% subplot(2,1,1)
% plot(t(11000:13000),legPosRight(11000:13000),t(11000:13000),legPosLeft(11000:13000))
% pR=polyfit(t(6200:7100),legPosRight(6200:7100),2);
% pL=polyfit(t(6200:7100),legPosLeft(6200:7100),2);
% lpRline=pR(1)*t(6200:7100).^2+pR(2)*t(6200:7100)+pR(3);
% lpLline=pL(1)*t(6200:7100).^2+pL(2)*t(6200:7100)+pL(3);
% plot(t(6200:7100),legPosRight(6200:7100)-lpRline,t(6200:7100),legPosLeft(6200:7100)-lpLline)
lpRtrend=movingAve(legPosRight,100);
lpLtrend=movingAve(legPosLeft,100);
lfRtrend=movingAve(freqRight,100);
lfLtrend=movingAve(freqLeft,100);
%plot(t,legPosRight-lpRtrend,t,legPosLeft-lpLtrend)
gyroX=Robot.gyroX;
rollInt=cumtrapz(t,gyroX);
intTrend=movingAve(rollInt,200);
% plot(t(1:3000),[legPosRight(1:3000)-legPosRight(1)
% legPosLeft(1:3000)-legPosLeft(1)])

plot(t,mod(legPosRight,2*pi),t,mod(legPosLeft,2*pi),t,...
    legPosRight-legPosLeft-min(legPosRight-legPosLeft),'r','linewidth',2)
xlim([0 max(t)])
ylim([-0.1 2*pi+0.1])
ylabel({'With springs:','Leg pos (rad)'})
legend('Right','Left','\theta_{R}-\theta_{L}')

%  plot(t,[legPosRight-legPosLeft-(legPosRight(1)-legPosLeft(1))...
%      50*(rollInt-intTrend)])  
 gyroX=Robot.gyroX;
 rollInt=cumtrapz(t,gyroX);
 intTrend=movingAve(rollInt,200);
% [ha,h1,h2]=plotyy(t,[mod(legPosRight/2/pi,1) mod(legPosLeft/2/pi,1)],t,180/pi*(rollInt-intTrend));%mod(legPosRight/2/pi,1)-mod(legPosLeft/2/pi,1));
% set(ha(1),'xlim',[0 max(t)],'ylim',[0 1])
% set(ha(2),'xlim',[0 max(t)],'ylim',[-10 10])
grid off
% legend('Right','Left','DCR','DCL')
% legend('Right','Left')
% xlabel('Time (s)')
% ylabel({'\theta (rad)'})
%set(gca,'xtick',0:.2:ttotal)

% ylim([-1 10])

% subplot(2,1,2)
%  gyroX=Robot.gyroX;
% rollInt=cumtrapz(t,gyroX);
% intTrend=movingAve(rollInt,200);
% rollFilt=filter(1,[1 -.9995],gyroX);
% plot(t,180/pi*(rollInt-intTrend),t,movingAve(180/pi*(rollInt-intTrend),50))
% grid on
% %set(gca,'xtick',0:.2:ttotal)
% legend('Actual','MA')
% ylabel('Roll (deg)')
% xlim([0 max(t)])


% legPosDiff=legPosRight-legPosLeft;
% plot(t,legPosDiff/max(abs(legPosDiff)),t,movingAve(mod(legPosDiff/2/pi,1),1))
% %plot(t,movingAve(mod(legPosDiff/2/pi,1),100))
% ylabel({'Left/right side leg position difference','(cycles)'})

% mean(legPosDiff(9000:17000))

BEMFRsh=BEMFR-min(BEMFR);
BEMFR1=BEMFRsh/max(BEMFRsh);
BEMFLsh=BEMFL-min(BEMFL);
BEMFL1=BEMFLsh/max(BEMFLsh)+1;

span = 50;
freqRightMA=movingAve(freqRight,span);
freqLeftMA=movingAve(freqLeft,span);
span = 250;
freqRightMA2=movingAve(freqRight,span);
freqLeftMA2=movingAve(freqLeft,span);

figure(3)
subplot(2,1,1)
plot(t(1:end-1),[freqRightMA freqLeftMA])
legend('right','left','location','southeast')
%plot(t,legPosRight,t,legPosLeft,t,DCR*560+515)
%legend('Phase diff.','Duty cycle','location','southeast')
% colormap([cool(32);autumn(32)]);
% scatter(t(1:end-1),freqRightMA,2,BEMFR1(1:end-1),'+')
% hold on
% scatter(t(1:end-1),freqLeftMA,2,BEMFL1(1:end-1),'x')
% hold off
xlabel('Time (s)')
ylabel('Leg frequency (Hz)');%\theta_{R}-\theta_{L} (rad)')
% ylim([-2 4])
xlim([12.5 14.5])%([0 max(t)])
grid off
% hold on
% plot(t,DCR*10+10.9,t,DCL*10+10.9)
% hold off
% DCR(10000)-DCL(10000)
% Torque1 = DCL(6500).*1.41.*(vBatt(6500)-BEMFL(6500))/3.3;
% Torque2 = DCL(12500).*1.41.*(vBatt(12500)-BEMFL(12500))/3.3;
% Tdiff=Torque2-Torque1
% TorquePeak = 1.41.*(890)/3.3
% Tpercent=(Torque2-Torque1)/TorquePeak
%set(gca,'xtick',0:2:ttotal)



gyroX=Robot.gyroX;
gyroY=Robot.gyroY;
gyroZ=Robot.gyroZ;
subplot(2,1,2)
%  gyroX=Robot.gyroX;
% rollInt=cumtrapz(t,gyroX);
% intTrend=movingAve(rollInt,100);
% rollFilt=filter(1,[1 -.9995],gyroX);
% plot(t(11000:13000),rollInt(11000:13000)-intTrend(11000:13000))
% grid on
% set(gca,'xtick',0:.2:ttotal)
% legend('x','y','z')
% ylabel('Gyro (rad/s)')
% xlim([11 13])

% Leg/body relative motion: explains suspended sync
workTogetherR=-freqRight.*cos(legPosRight(1:end-1)).*gyroX(1:end-1);
workTogetherL=freqLeft.*cos(legPosLeft(1:end-1)).*gyroX(1:end-1);

% plot(t(1:end-1),movingAve(workTogetherR,80),'k',...
%     t(1:end-1),movingAve(workTogetherL,80),'r',[-10 50],[0 0],'k:')

zmRollAngle=180/pi*(rollInt-intTrend); %zero mean
%plot(t(1:end-1),freqRightMA+zmRollAngle(1:end-1))
plot(t,mod(legPosRight/2/pi-legPosLeft/2/pi,1),'r')
xlim([12.5 14.5])%[0 max(t)])
grid off

% freqDiff=movingAve(freqRightMA-freqLeftMA,200);
% plot(t(1:end-1),freqDiff)
% legend('Right','Left')
% ylabel({'Legs/body relative motion', '(sign only)'})
%ylabel('FreqR+zmRollAngle')
xlabel('Time (s)')
ylabel('\theta_R-\theta_L (cycles)')
ylim([0 1.3])
%legend('Right','Left')
% xlabel('DC (%)')

figure(4)
subplot(2,1,1)
plot(t,gyroX,t,gyroY,t,gyroZ)
legend('x','y','z')
ylabel('Gyro (rad/s)')
xlim([0 max(t)])
ylim([-3 3])
%set(gca,'xtick',0:2:ttotal)
subplot(2,1,2)
plot(t,torque1(:,1),t,torque1(:,2))
xlim([0 max(t)])
ylim([-0.5 1])
ylabel('Torque (mN*m)')


% xlX=Robot.xlX;
% xlY=Robot.xlY;
% xlZ=Robot.xlZ;
% subplot(3,1,3)
% plot(t,xlX,t,xlY,t,xlZ)
% legend('x','y','z')
% ylabel('Acceleration (m/s^2)')

 freqLeftAve=mean(freqLeft(1000:end))
 freqRightAve=mean(freqRight(1000:end))
 %% nice figure
 figure(5)
ha = tight_subplot(3,1,[.01 .03],[.1 .03],[.12 .08]);
 axes(ha(1))
 plot(t,(legPosRight-legPosLeft)/2/pi)
 ylabel('\theta_{R}-\theta_{L} (cycles)')
xlim([0 max(t)])
ylim([-1 4])
set(ha(1),'xticklabel',[])

axes(ha(2))
plot(t(1:end-1),freqRightMA,t(1:end-1),freqLeftMA)
legend('Right','Left','location','southeast')
ylabel({'Motor frequency ','(Hz)'})
ylim([1 7])
xlim([0 max(t)])
set(ha(2),'xticklabel',[])

axes(ha(3))
[ha,h1,h2]=plotyy(t,torque1(:,1),t,(rollInt-intTrend)*180/pi);
xlim([0 max(t)])
xlabel('Time (s)')
ylabel(ha(1),{'Right motor', 'torque (mN*m)'})
ylabel(ha(2),'Body roll angle (degrees)')
linkaxes(ha,'x')
set(ha(1),'ylim',[70 165],'ycolor','k')
set(ha(2),'ylim',[-15 15],'ycolor','r')
set(h1,'color','k')
set(h2,'color','r')


%%
%more torque vs. phase: static ground tests
clear all
span=25;
[legPos1,freq1,gyro1,torque1,t1,Robot]=getLegMovement('L2to6Hz_R1400_weak.txt',span);
rangeNo=[2000:4000];
rangeYes=[5000:7000];
legPhase=mod(legPos1/2/pi,1);
gyroX=Robot.gyroX;
rollInt=cumtrapz(t1,gyroX);
intTrend=movingAve(rollInt,200);
rollAngle=rollInt-intTrend;

figure(5)
subplot(2,1,1)
plot(rollAngle(rangeNo),legPhase(rangeNo,1),'.',rollAngle(rangeNo),legPhase(rangeNo,2),'.')
xlim([-.15 .1])
% plot3(legPhase(rangeNo,1),freq1(rangeNo,3),rollAngle(rangeNo),'.',legPhase(rangeNo,2),freq1(rangeNo,4),rollAngle(rangeNo),'.')
% xlim([0 2*pi])
ylabel('Leg phase (rad)')
ylim([0 1])
subplot(2,1,2)
plot(rollAngle(rangeYes),legPhase(rangeYes,1),'.',rollAngle(rangeYes),legPhase(rangeYes,2),'.')
xlabel('Body roll (degreesish?)')
xlim([-.15 .1])
% plot3(legPhase(rangeYes,1),freq1(rangeYes,3),rollAngle(rangeYes),'.',legPhase(rangeYes,2),freq1(rangeYes,4),rollAngle(rangeYes),'.')
% xlim([0 2*pi])
% xlabel('Leg phase (rad)')
ylabel('Leg phase (rad)')
ylim([0 1])


figure(6)
subplot(2,1,1)
colormap([copper(32);jet(32)])
scatter(rollAngle(rangeNo),freq1(rangeNo,3),4,legPhase(rangeNo,1))
hold on
scatter(rollAngle(rangeNo),freq1(rangeNo,4),4,legPhase(rangeNo,2)+1)
hold off
ylabel('Leg frequency (rad/s)')
subplot(2,1,2)
scatter(rollAngle(rangeYes),freq1(rangeYes,3),4,legPhase(rangeYes,1))
hold on
scatter(rollAngle(rangeYes),freq1(rangeYes,4),4,legPhase(rangeYes,2)+1)
hold off
ylabel('Leg frequency (rad/s)')
xlabel('Body roll')

rangeAll=1:21000;
rollAngle(rangeAll)
figure(7)
colormap([jet(64)])
% colormap([copper(32);jet(32)])
scatter(t1(rangeAll),legPhase(rangeAll,2),4,'k')%+range(rollAngle(rangeAll)))
hold on
scatter(t1(rangeAll),legPhase(rangeAll,1),4,rollAngle(rangeAll),'filled')
plot(t1(1:end-1),freq1(:,3:4)/max(freq1(:,3))+1.2,t1,rollAngle/2/max(abs(rollAngle))+1.6,'r')
hold off
xlim([0 max(t1)])
grid on

j=1;
k=1;
for i=1:length(t1)
    if legPhase(i,1)<.8 && legPhase(i,1)>.7
        contactRollR(j)=rollAngle(i);
        tContactR(j)=t1(i);
        j=j+1;
    end
    if legPhase(i,2)<.8 && legPhase(i,2)>.7
        contactRollL(k)=rollAngle(i);
        tContactL(k)=t1(i);
        k=k+1;
    end
end
length(tContactR)
length(tContactL)
x=linspace(0,max(t1),5000);
linCRR=interp1(tContactR,movingAve(contactRollR,50),x);
linCRL=interp1(tContactL,movingAve(contactRollL,50),x);
figure(8)
subplot(2,1,1)
plot(tContactR,movingAve(contactRollR,50),'.-',tContactL,movingAve(contactRollL,50),'.-')
% plot(tContactR,contactRollR,'.-',tContactL,contactRollL,'.-')
subplot(2,1,2)
plot(x,linCRR-linCRL)
%%
%figures for lab meeting 5/1
clear all
span=25;
[legPos1,freq1,gyro1,torque1,t1,Robot]=getLegMovement('L3to5Hz_R1400_weak.txt',span);
legPhase=mod(legPos1/2/pi,1);
gyroX=Robot.gyroX;
rollInt=cumtrapz(t1,gyroX);
intTrend=movingAve(rollInt,200);
rollAngle=rollInt-intTrend;

figure(7)
ha = tight_subplot(2,1,[.01 .03],[.1 .03],[.12 .08]);
axes(ha(1))
[haxes,h1,h2]=plotyy(t1(1:end-1),freq1(:,3:4),t1,rollAngle*180/pi);
set(h1(2),'color','k')
set(h1(1),'linewidth',2)
set(h2,'color','r','linestyle','--','linewidth',2)
linkaxes(haxes,'x')
set(haxes(1),'ylim',[0 10])
set(haxes(2),'ycolor','k','ylim',[-10 10])
xlim([13 15])
ylabel(haxes(1),'Leg frequency (Hz)')
ylabel(haxes(2),'Roll angle (degrees)')
hl=legend('$\dot{\theta}_{R}$','$\dot{\theta}_{L}$','Roll');
set(hl,'Interpreter','latex')
set(haxes,'xticklabel',[])

axes(ha(2))
colormap([cool(64)])
% colormap([copper(32);jet(32)])
scatter(t1,legPhase(:,2),3,'k','filled')%+range(rollAngle(rangeAll)))
hold on
scatter(t1,legPhase(:,1),4,rollAngle*180/pi,'filled')
hold off
xlim([13 15])
grid on
ylabel('Leg position within cycle')
xlabel('Time (s)')
legend('Left','Right')

span2=100;
freqRMA=movingAve(freq1(:,1),span2);
freqLMA=movingAve(freq1(:,2),span2);
figure(8)
ha2 = tight_subplot(2,1,[.01 .03],[.1 .03],[.12 .08]);
axes(ha2(1))
plot(t1,(legPos1(:,1)-legPos1(:,2))/2/pi,'r')
xlim([0 max(t1)])
ylabel({'Difference in leg', 'position (cycles)'})
set(haxes,'xticklabel',[])

axes(ha2(2))
plot(t1(1:end-1),freqRMA,t1(1:end-1),freqLMA,'k')
xlim([0 max(t1)])
xlabel('Time (s)')
ylabel({'Leg frequency','(moving ave.)(Hz)'})
legend('Right','Left','location','southeast')

%%
%run above cell first
span2=50;
freqRMA=movingAve(freq1(:,1),span2);
freqLMA=movingAve(freq1(:,2),span2);

figure(9)
rangeS=4800:5800;
rangeU=6000:7000;
subplot(2,1,1)
plot(legPhase(rangeU,1),rollAngle(rangeU)*180/pi,'.',legPhase(rangeU,2),rollAngle(rangeU)*180/pi,'.',...
    legPhase(rangeU,1),freqRMA(rangeU)+5,'c.',legPhase(rangeU,2),freqLMA(rangeU)+5,'g.')
ylim([-8 10])
ylabel({'Unsynchronized','Roll angle (degrees)'})
legend('Right (constant PWM)','Left (controlled)')
subplot(2,1,2)
hold on
plot(legPhase(rangeS,1),rollAngle(rangeS)*180/pi,'.',legPhase(rangeS,2),rollAngle(rangeS)*180/pi,'.',...
    legPhase(rangeS,1),freqRMA(rangeS)+5,'c.',legPhase(rangeS,2),freqLMA(rangeS)+5,'g.')
ylim([-8 10])
ylabel({'Synchronized','Roll angle (degrees)'})
xlabel('Leg phase')

%%
%MORE torque vs phase 4/21/15
span=50;
[legPos,freq,gyro,torque,t,Robot]=getLegMovement('L3to5Hz_R1400_weak.txt',span);
legPhase=mod(legPos/2/pi,1);
gyroX=Robot.gyroX;
rollInt=cumtrapz(t,gyroX);
intTrend=movingAve(rollInt,200);
rollAngle=(rollInt-intTrend)*180/pi;

range1=3400:4400; %1
range2=7000:8000; %2
range3=10000:11000; %3
range4=15000:16000; %4
range5=19000:20000; %5
figure(6)
subplot(5,1,1)
[ax1 h11 h21]=plotyy(legPhase(range1,:),freq(range1,3:4),legPhase(range1,:),[rollAngle(range1) rollAngle(range1)]);
set(ax1(1),'ylim',[-2 9])
set(ax1(2),'ylim',[-10 20],'ycolor','k')
set(h11,'linestyle','none','marker','.')
set(h21(1),'linestyle','none','marker','.','color','c')
set(h21(2),'linestyle','none','marker','.','color','g')

subplot(5,1,2)
[ax2 h12 h22]=plotyy(legPhase(range2,:),freq(range2,3:4),legPhase(range2,:),[rollAngle(range2) rollAngle(range2)]);
set(ax2(1),'ylim',[-2 9])
set(ax2(2),'ylim',[-10 20],'ycolor','k')
set(h12,'linestyle','none','marker','.')
set(h22(1),'linestyle','none','marker','.','color','c')
set(h22(2),'linestyle','none','marker','.','color','g')

subplot(5,1,3)
[ax3 h13 h23]=plotyy(legPhase(range3,:),freq(range3,3:4),legPhase(range3,:),[rollAngle(range3) rollAngle(range3)]);
set(ax3(1),'ylim',[-2 9])
set(ax3(2),'ylim',[-10 20],'ycolor','k')
set(h13,'linestyle','none','marker','.')
set(h23(1),'linestyle','none','marker','.','color','c')
set(h23(2),'linestyle','none','marker','.','color','g')

subplot(5,1,4)
[ax4 h14 h24]=plotyy(legPhase(range4,:),freq(range4,3:4),legPhase(range4,:),[rollAngle(range4) rollAngle(range4)]);
set(ax4(1),'ylim',[-2 9])
set(ax4(2),'ylim',[-10 20],'ycolor','k')
set(h14,'linestyle','none','marker','.')
set(h24(1),'linestyle','none','marker','.','color','c')
set(h24(2),'linestyle','none','marker','.','color','g')

subplot(5,1,5)
[ax5 h15 h25]=plotyy(legPhase(range5,:),freq(range5,3:4),legPhase(range5,:),[rollAngle(range5) rollAngle(range5)]);
set(ax5(1),'ylim',[-2 9])
set(ax5(2),'ylim',[-10 20],'ycolor','k')
set(h15,'linestyle','none','marker','.')
set(h25(1),'linestyle','none','marker','.','color','c')
set(h25(2),'linestyle','none','marker','.','color','g')
%plot(legPhase(range5,:),freq(range5,3:4),'.')
%ylim([1 7])

%%
%weird looking for gyro/position trends
%must run previous cell first
figure(5)
t1=t(600:end-100);
gyro0=rollInt-intTrend;
gyro0sh=gyro0-min(gyro0);
gyro01=gyro0sh/max(gyro0sh);

 lpR0=legPosRight-lpRtrend;
 lpR0sh=lpR0(600:end-100)-min(lpR0(600:end-100));
 lpR01=lpR0sh/max(lpR0sh);
 lpL0=legPosLeft-lpLtrend;
 lpL0sh=lpL0(600:end-100)-min(lpL0(600:end-100));
 lpL01=lpL0sh/max(lpL0sh);
 
 lfR0=freqRight-lfRtrend;
 lfR0sh=lfR0(600:end-99)-min(lfR0(600:end-99));
 lfR01=lfR0sh/max(lfR0sh);
 lfL0=freqLeft-lfLtrend;
 lfL0sh=lfL0(600:end-99)-min(lfL0(600:end-99));
 lfL01=lfL0sh/max(lfL0sh);
 
  subplot(3,1,1)
  length(t)
  length(freqRightMA)
%   plot(t1,freqRightMA(600:end-99),t1,freqLeftMA(600:end-99))
plot(legPosRight(1:end-1),freqRightMA,legPosRight(1:end-1),freqLeftMA)
  ylim([15 20])
  subplot(3,1,2)
%   plot(t1,lfR01-gyro01)
%   ylabel('RIGHT')
%   ylim([-1 1])
%plot freq & roll vs position, mark every cycle
plot(legPosRight(1:end-1),freqRight-lfRtrend,legPosRight(1:end),gyro01)
x=0:2*pi:3500;
hold on
for k=1:length(x)
    plot([x(k) x(k)],[-20 20],'k:')
end
hold off
  subplot(3,1,3)
%   plot(t1,lfL01-gyro01-(lfR01-gyro01))
%   ylabel('LEFT')
%   ylim([-1 1])
plot(legPosLeft(1:end-1),freqLeft-lfLtrend,legPosLeft(1:end),gyro01)
x=0:2*pi:3500;
hold on
for k=1:length(x)
    plot([x(k) x(k)],[-20 20],'k:')
end
hold off
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
span=50;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('sus_torqueVsPhase_tri_static.txt',span);
[legPos2,freq2,gyro2,torque2,t2,Robot2]=getLegMovement('ground_torqueVsPhase_tri_static.txt',span);

legPhase1=mod(legPos1,2*pi);
legPhase2=mod(legPos2,2*pi);

figure(1)

%torque from crank
subplot(3,1,1)
plot(legPhase1(:,1),torque1(:,1),'.',...
    legPhase1(:,2),torque1(:,2),'.',[0 10],[0 0],'k-')
title('Crank/Flexures')
axis([0 2*pi -150 200])
ylabel('Torque')

%torque from ground+crank
subplot(3,1,2)
plot(legPhase2(:,1),torque2(:,1),'.',...
    legPhase2(:,2),torque2(:,2),'.',[0 10],[0 0],'k-')
title('Ground + Crank/Flexures')
axis([0 2*pi -150 200])
ylabel('Torque')

%torque from ground alone
legPhase3=(legPhase1+legPhase2)/2;
torque3=torque2-torque1;
subplot(3,1,3)
plot(legPhase3(:,1),torque3(:,1),'.',legPhase3(:,2),torque3(:,2),'.',[0 10],[0 0],'k-')
title('Ground Alone')
axis([0 2*pi -150 200])
ylabel('Torque')
xlabel('Leg phase (rad)')

span=50;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('sus_torqueVsPhase_zero_static.txt',span);
[legPos2,freq2,gyro2,torque2,t2,Robot2]=getLegMovement('ground_torqueVsPhase_zero_static.txt',span);

legPhase1=mod(legPos1,2*pi);
legPhase2=mod(legPos2,2*pi);

figure(2)

%torque from crank
subplot(3,1,1)
plot(legPhase1(:,1),torque1(:,1),'.',...
    legPhase1(:,2),torque1(:,2),'.',[0 10],[0 0],'k-')
title('Crank/Flexures')
axis([0 2*pi -150 200])
ylabel('Torque')

%torque from ground+crank
subplot(3,1,2)
plot(legPhase2(:,1),torque2(:,1),'.',...
    legPhase2(:,2),torque2(:,2),'.',[0 10],[0 0],'k-')
title('Ground + Crank/Flexures')
axis([0 2*pi -150 200])
ylabel('Torque')

%torque from ground alone
legPhase3=(legPhase1+legPhase2)/2;
torque3=torque2-torque1;
subplot(3,1,3)
plot(legPhase3(:,1),torque3(:,1),'.',legPhase3(:,2),torque3(:,2),'.',[0 10],[0 0],'k-')
title('Ground Alone')
axis([0 2*pi -150 200])
ylabel('Torque')
xlabel('Leg phase (rad)')
%%
%more torque vs phase

span=50;
[legPos1,freq1,gyro1,torque1,t1,Robot1]=getLegMovement('sus_torqueVsPhase_Rwave_2500.txt',span);
[legPos2,freq2,gyro2,torque2,t2,Robot2]=getLegMovement('ground_torqueVsPhase_Rwave_2500.txt',span);

legPhase1=mod(legPos1,2*pi);
legPhase2=mod(legPos2,2*pi);
DCR1=Robot1.DCR;
DCL1=Robot1.DCL;
DCR2=Robot2.DCR;
DCL2=Robot2.DCL;

figure(1)

%torque from crank
%plot 1/2 second raw data
subplot(3,1,1)
%note: right side leads
plot(legPhase1(13500:14000,1),torque1(13500:14000,1)./DCR1(13500:14000),'.',...
    legPhase1(13500:14000,2),torque1(13500:14000,2)./DCL1(13500:14000),'.',[0 10],[0 0],'k-')
title('Synchronized: Crank/Flexures')
axis([0 2*pi 200 400])
ylabel('Torque')
hold on

%sort each side by phase
sorterR1=[legPhase1(13500:14000,1) torque1(13500:14000,1) DCR1(13500:14000)];
[~,I]=sort(sorterR1(:,1));
sorterR1=sorterR1(I,:);
lpR1sort=sorterR1(:,1);
tR1sort=sorterR1(:,2);
DCR1sort=sorterR1(:,3);
sorterL1=[legPhase1(13500:14000,2) torque1(13500:14000,2) DCL1(13500:14000)];
[~,I]=sort(sorterL1(:,1));
sorterL1=sorterL1(I,:);
lpL1sort=sorterL1(:,1);
tL1sort=sorterL1(:,2);
DCL1sort=sorterL1(:,3);

%clean data (moving average)
tR1ave=movingAve(tR1sort./DCR1sort,5);
tL1ave=movingAve(tL1sort./DCL1sort,5);
%vector for interpolation (want all data sets to have same sampling points)
lp1int=linspace(0,2*pi,500);
%remove repeated sampling points for interpolation
[lpR1sort, ind]=unique(lpR1sort);
tR1ave=tR1ave(ind);
[lpL1sort, ind]=unique(lpL1sort);
tL1ave=tL1ave(ind);
%interpolate
tR1int=interp1(lpR1sort,tR1ave,lp1int);
tL1int=interp1(lpL1sort,tL1ave,lp1int);

%plot sorted & interpolated data
plot(lpR1sort,tR1ave,'c',...
    lpL1sort,tL1ave,'g','linewidth',2)
plot(lp1int,tR1int,'r:',lp1int,tL1int,'r:')
hold off

%torque from ground+crank
subplot(3,1,2)
%note: DC's matched
plot(legPhase2(16000:16500,1),torque2(16000:16500,1)./DCR2(16000:16500),'.',...
    legPhase2(16000:16500,2),torque2(16000:16500,2)./DCL2(16000:16500),'.',[0 10],[0 0],'k-')
title('Ground + Crank/Flexures')
axis([0 2*pi 200 400])
ylabel('Torque')
hold on

sorterR2=[legPhase2(16000:16500,1) torque2(16000:16500,1) DCR2(16000:16500)];
[~,I]=sort(sorterR2(:,1));
sorterR2=sorterR2(I,:);
lpR2sort=sorterR2(:,1);
tR2sort=sorterR2(:,2);
DCR2sort=sorterR2(:,3);
sorterL2=[legPhase2(16000:16500,2) torque2(16000:16500,2) DCL2(16000:16500)];
[~,I]=sort(sorterL2(:,1));
sorterL2=sorterL2(I,:);
lpL2sort=sorterL2(:,1);
tL2sort=sorterL2(:,2);
DCL2sort=sorterL2(:,3);

tR2ave=movingAve(tR2sort./DCR2sort,10);
tL2ave=movingAve(tL2sort./DCL2sort,10);
lp2int=linspace(0,2*pi,500);
[lpR2sort, ind]=unique(lpR2sort);
tR2ave=tR2ave(ind);
[lpL2sort, ind]=unique(lpL2sort);
tL2ave=tL2ave(ind);
tR2int=interp1(lpR2sort,tR2ave,lp2int);
tL2int=interp1(lpL2sort,tL2ave,lp2int);

plot(lpR2sort,tR2ave,'c',...
    lpL2sort,tL2ave,'g','linewidth',2)
plot(lp2int,tR2int,'r:',lp2int,tL2int,'r:')
hold off

%torque from ground alone
lp3=lp2int;
tR3=tR2int-tR1int;
tL3=tL2int-tL1int;
subplot(3,1,3)
plot(lp3,tR3,'c',lp3,tL3,'g','linewidth',2)
title('Ground Alone')
axis([0 2*pi -50 200])
ylabel('Torque')
xlabel('Leg phase (rad)')


figure(2)

%torque from crank
subplot(3,1,1)
%note:right side lags
plot(legPhase1(3000:3500,1),torque1(3000:3500,1)./DCR1(3000:3500),'.',...
    legPhase1(3000:3500,2),torque1(3000:3500,2)./DCL1(3000:3500),'.',[0 10],[0 0],'k-')
title('Non-synchronized: Crank/Flexures')
axis([0 2*pi 200 400])
ylabel('Torque')
hold on

sorterR4=[legPhase1(3000:3500,1) torque1(3000:3500,1) DCR1(3000:3500)];
[~,I]=sort(sorterR4(:,1));
sorterR4=sorterR4(I,:);
lpR4sort=sorterR4(:,1);
tR4sort=sorterR4(:,2);
DCR4sort=sorterR4(:,3);
sorterL4=[legPhase1(3000:3500,2) torque1(3000:3500,2) DCL1(3000:3500)];
[~,I]=sort(sorterL4(:,1));
sorterL4=sorterL4(I,:);
lpL4sort=sorterL4(:,1);
tL4sort=sorterL4(:,2);
DCL4sort=sorterL4(:,3);

tR4ave=movingAve(tR4sort./DCR4sort,5);
tL4ave=movingAve(tL4sort./DCL4sort,5);
lp4int=linspace(0,2*pi,500);
[lpR4sort, ind]=unique(lpR4sort);
tR4ave=tR4ave(ind);
[lpL4sort, ind]=unique(lpL4sort);
tL4ave=tL4ave(ind);
tR4int=interp1(lpR4sort,tR4ave,lp4int);
tL4int=interp1(lpL4sort,tL4ave,lp4int);

plot(lpR4sort,tR4ave,'c',...
    lpL4sort,tL4ave,'g','linewidth',2)
plot(lp4int,tR4int,'r:',lp4int,tL4int,'r:')
hold off

%torque from ground+crank
subplot(3,1,2)
%note: right side lags
plot(legPhase2(20500:21000,1),torque2(20500:21000,1)./DCR2(20500:21000),'.',...
    legPhase2(20500:21000,2),torque2(20500:21000,2)./DCL2(20500:21000),'.',[0 10],[0 0],'k-')
title('Ground + Crank/Flexures')
axis([0 2*pi 200 400])
ylabel('Torque')
hold on

sorterR5=[legPhase2(20500:21000,1) torque2(20500:21000,1) DCR2(20500:21000)];
[~,I]=sort(sorterR5(:,1));
sorterR5=sorterR5(I,:);
lpR5sort=sorterR5(:,1);
tR5sort=sorterR5(:,2);
DCR5sort=sorterR5(:,3);
sorterL5=[legPhase2(20500:21000,2) torque2(20500:21000,2) DCL2(20500:21000)];
[Y,I]=sort(sorterL5(:,1));
sorterL5=sorterL5(I,:);
lpL5sort=sorterL5(:,1);
tL5sort=sorterL5(:,2);
DCL5sort=sorterL5(:,3);

tR5ave=movingAve(tR5sort./DCR5sort,5);
tL5ave=movingAve(tL5sort./DCL5sort,5);
lp5int=linspace(0,2*pi,500);

[lpR5sort, ind]=unique(lpR5sort);
tR5ave=tR5ave(ind);
[lpL5sort, ind]=unique(lpL5sort);
tL5ave=tL5ave(ind);

tR5int=interp1(lpR5sort,tR5ave,lp5int);
tL5int=interp1(lpL5sort,tL5ave,lp5int);

plot(lpR5sort,tR5ave,'c',...
    lpL5sort,tL5ave,'g','linewidth',2)
plot(lp5int,tR5int,'r:',lp5int,tL5int,'r:')
hold off


%torque from ground alone
lp6=lp5int;
tR6=tR5int-tR4int;
tL6=tL5int-tL4int;
subplot(3,1,3)
plot(lp6,tR6,'c',lp6,tL6,'g','linewidth',2)
title('Ground Alone')
axis([0 2*pi -50 200])
ylabel('Torque')
xlabel('Leg phase (rad)')

figure(3)
subplot(3,1,1)
plot(lp1int,tR1int-tR4int,'k',lp1int,tL1int-tL4int,'r')
axis([0 2*pi -45 45])
ylabel('\tau_{sync}-\tau_{non}')
title('Crank/Flexures')
subplot(3,1,2)
plot(lp2int,tR2int-tR5int,'k',lp2int,tL2int-tL5int,'r')
axis([0 2*pi -45 45])
ylabel('\tau_{sync}-\tau_{non}')
title('Ground + Crank/Flexures')
subplot(3,1,3)
plot(lp2int,tR3-tR6,'k',lp2int,tL3-tL6,'r')
axis([0 2*pi -45 45])
xlabel('Leg phase (rad)')
ylabel('\tau_{sync}-\tau_{non}')
title('Ground Alone')

%%
%looking at BEMF, frequency, gyro
span=50;
[legPos1,freq1,gyro1,torque1,t1,Robot]=getLegMovement('sus_Rwave_1850_3150.txt',span);
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
roll=Robot.gyroX;
rollInt(1)=0;
for i=1:length(roll)-1
    int(i)=trapz(t(i:i+1),roll(i:i+1));
    rollInt(i+1)=rollInt(i)+int(i);
end
rollInt=cumtrapz(t,roll);
pitch=Robot.gyroY;
yaw=Robot.gyroZ;

BEMFR=movingAve(BEMFR,1);
BEMFL=movingAve(BEMFL,1);

BEMFRsh=BEMFR-min(BEMFR);
BEMFR1=BEMFRsh/max(BEMFRsh);
BEMFLsh=BEMFL-min(BEMFL);
BEMFL1=BEMFLsh/max(BEMFLsh)+1;

torqueRsh=torque1(:,1)-min(torque1(:,1));
torqueR1=torqueRsh/max(torqueRsh);
torqueLsh=torque1(:,2)-min(torque1(:,2));
torqueL1=torqueLsh/max(torqueLsh)+1;

tStep=t(2)-t(1);
freqRight=diff(legPosRight)/tStep/(2*pi);
freqLeft=diff(legPosLeft)/tStep/(2*pi);
span = 150;
freqRightMA=movingAve(freqRight,span);
freqLeftMA=movingAve(freqLeft,span);
% figure(1)
% subplot(2,1,1)
% %plot(t(1:end-1),freqRightMA,'k',t(1:end-1),freqLeftMA,'r')
% colormap([cool(32);autumn(32)]);
% scatter(t(1:end-1),freqRightMA,2,BEMFR1(1:end-1),'+')
% hold on
% scatter(t(1:end-1),freqLeftMA,2,BEMFL1(1:end-1),'x')
% plot(t,roll)
% hold off
% legend('Right side','Left side')
% xlabel('Time (s)')
% ylabel('Leg frequency (Hz)')
% set(gca,'xtick',0:2:ttotal)

figure(1)
colormap([cool(32);autumn(32)]);
[AX,H1,H2]=plotyy(t,rollInt,t(1:end-1),freqRightMA,@plot,@(x,y) scatter(x,y,4,BEMFR1(1:end-1)));%torqueR1(1:end-1)));
hold(AX(2),'on')
scatter(AX(2),t(1:end-1),freqLeftMA,4,BEMFL1(1:end-1),'filled');%torqueL1(1:end-1));
hold(AX(1),'on')
plot(t,DCR*15+20,'k',t,DCL*15+20,'r')
set(AX(1),'ylim',[-5 5])
set(AX(2),'ylim',[13 22])

figure(2)
BEMFR=movingAve(BEMFR,50);
BEMFL=movingAve(BEMFL,50);
plot(t,BEMFR,t,BEMFL)%,t,torque1)
ylim([50 250])





