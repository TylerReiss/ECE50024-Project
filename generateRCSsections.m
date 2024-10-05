clear;
close all;

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

projectDir = 'C:\Users\tyler\OneDrive\Documents\College\Graduate\Machine Learning\Project\Code';

c = physconst('Lightspeed');
fc = 4.5e9;
radbot = 0.001;
radtop = 1;
hgt    = 3;
az = 0;
el = -89.5:1:89.5;

linewidth = 1.5;
fontSize = 18;
axFontSize = 12;

figure
[rcspat,~,elresp] = rcstruncone(radbot,radtop,hgt,c,fc,az,el);
rcsConedBsm = pow2db(rcspat(:,1));
plot(elresp+90,rcsConedBsm,'k-','LineWidth',linewidth)
title('Cone RCS Distribution','FontSize',fontSize)
grid on

ax = gca;
ax.XAxis.FontSize = axFontSize;
ax.YAxis.FontSize = axFontSize;
xlabel('Aspect Angle (deg)','FontSize',fontSize)
ylabel('RCS (dBsm)','FontSize',fontSize)
xlim([0 180])

figure
plot(elresp+90,elresp+90,'k-','LineWidth',linewidth)
title('Ideal RCS Distribution','FontSize',fontSize)
grid on

ax = gca;
ax.XAxis.FontSize = axFontSize;
ax.YAxis.FontSize = axFontSize;
xlabel('Aspect Angle (deg)','FontSize',fontSize)
ylabel('RCS (dBsm)','FontSize',fontSize)
xlim([0 180])

figure
[rcspat,azresp,elresp] = rcscylinder(radtop,radtop,hgt,c,fc,az,el);
rcsCylinderdBsm = pow2db(rcspat(:,1));
plot(elresp+90,rcsCylinderdBsm,'k-','LineWidth',linewidth)
title('Cylinder RCS Distribution','FontSize',fontSize)
grid on

ax = gca;
ax.XAxis.FontSize = axFontSize;
ax.YAxis.FontSize = axFontSize;
xlabel('Aspect Angle (deg)','FontSize',fontSize)
ylabel('RCS (dBsm)','FontSize',fontSize)
xlim([0 180])