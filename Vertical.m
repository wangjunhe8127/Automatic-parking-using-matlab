%% 
%运行的时候可以分节运行，按照顺序挨着运行，如果全部运行也行，但是运行后弹出的绘图窗口太多
%         omiga_fr(k,1) = (1/(1+(d_ym(k,1))^2)^(1/2))*diff(delta_fr(k,1),xm);
%%
clc
clear
close
%车辆参数   
L = 3.95;  %车长/m
W = 1.97;  %车宽/m
l = 2.48;   %轴距/m
lf = 0.8;    %前悬/m
lr = 0.67;   %后悬/m
delta_f = 0.524;  %前轮最大转角/rad
omiga_f = 0.524;  %前轮最大转角转速/rad/s 
Rmin = 4.3;  %最小转弯半径/m
h = 4;      %道路宽度/m
Lp = 7;   %车位长度/m
Wp = 2.2;    %车位宽度/m    
%起始点坐标
xM1 = 4.5;
yM1 = 0.8;
%中间点坐标
xM2 = 4.5;
yM2 = 4.5;
%末端点坐标
xM3 = 9.5;
yM3 = 9.5;     
%距离
DX32 = xM3-xM2;
DY32 = yM3-yM2;
DY21 = yM2-yM1;    
%首先定义各个变量
k = 1;
ya = [];yb = [];yc = [];yd = [];
xa = [];xb = [];xc = [];xd = [];
xmr = [];ym = [];ymr = [];
phi = [];d_ym = [];d2_ym = [];
tho = [];delta_fr = [];tmp = [];
for xm = 0:0.01:DY21
    xmr = [xmr;xM1];%线性移动
    ymr = [ymr;yM1+xm];%规划的y坐标
    phi = [phi;pi/2];%y坐标斜率的arctan可以得到角度
    d_ym = [d_ym;inf];%y坐标斜率
    d2_ym = [d2_ym;0];%y坐标斜率
    tho = [tho;d2_ym(k)/((1+d_ym(k)^2)^(3/2))];
    delta_fr = [delta_fr;atan(l*(d2_ym(k))/((1+d_ym(k)^2)^(3/2)))];
    %车辆各点坐标   
    xa=[xa;xmr(k)-lr*cos(phi(k))+W/2*sin(phi(k))];
    xb=[xb;xmr(k)+(L-lr)*cos(phi(k))+W/2*sin(phi(k))];
    xc=[xc;xmr(k)+(L-lr)*cos(phi(k))-W/2*sin(phi(k))];
    xd=[xd;xmr(k)-lr*cos(phi(k))-W/2*sin(phi(k))];
    ya=[ya;ymr(k)-lr*sin(phi(k))-W/2*cos(phi(k))];
    yb=[yb;ymr(k)+(L-lr)*sin(phi(k))-W/2*cos(phi(k))];
    yc=[yc;ymr(k)+(L-lr)*sin(phi(k))+W/2*cos(phi(k))];
    yd=[yd;ymr(k)-lr*sin(phi(k))+W/2*cos(phi(k))];
    k = k+1;
end
%圆弧中心
x0 = 9.5;
y0 = 4.5;
R = 5;
for xm = 0:0.01:DX32
    xmr = [xmr;xM1+xm];%线性移动
    ymr = [ymr;y0+(R^2-(xmr(k)-x0)^2)^(1/2)];%规划的y坐标
    phi = [phi;atan(-(xmr(k)-x0)/(ymr(k)-y0))];%y坐标斜率的arctan可以得到角度
    d_ym = [d_ym;-(xmr(k)-x0)/(ymr(k)-y0)];%y坐标斜率
    d2_ym = [d2_ym;(-1*(ymr(k)-y0)+d_ym(k)*(xmr(k)-x0))/(ymr(k)-y0)^2];%y斜率的二次方
    tho = [tho;d2_ym(k)/((1+d_ym(k)^2)^(3/2))];%曲率公式，表示弯曲程度
    delta_fr = [delta_fr;atan(l*(d2_ym(k))/((1+d_ym(k)^2)^(3/2)))];%等效前轮转角，见博客(注意有的是l有的是1)
    %车辆各点坐标   
    xa=[xa;xmr(k)-lr*cos(phi(k))+W/2*sin(phi(k))];
    xb=[xb;xmr(k)+(L-lr)*cos(phi(k))+W/2*sin(phi(k))];
    xc=[xc;xmr(k)+(L-lr)*cos(phi(k))-W/2*sin(phi(k))];
    xd=[xd;xmr(k)-lr*cos(phi(k))-W/2*sin(phi(k))];
    
    ya=[ya;ymr(k)-lr*sin(phi(k))-W/2*cos(phi(k))];
    yb=[yb;ymr(k)+(L-lr)*sin(phi(k))-W/2*cos(phi(k))];
    yc=[yc;ymr(k)+(L-lr)*sin(phi(k))+W/2*cos(phi(k))];
    yd=[yd;ymr(k,1)-lr*sin(phi(k,1))+W/2*cos(phi(k,1))];
    k = k+1;
end
omiga_fr = [0;diff(d2_ym)/0.01];
%%
% 绘图
% 车辆各点路径图
figure
box off
set(0,'defaultfigurecolor','w')
hold on

color01 = [1,0.5,0];
color03 = [0,1,1];
color04 = [1,1,0];

plot(xa,ya,'color',color01,'linewidth',2)
plot(xb,yb,'b','linewidth',2)
plot(xc,yc,'color',color03,'linewidth',2)
plot(xd,yd,'color',color04,'linewidth',2)
plot(xmr,ymr,'k','linewidth',2)


% 车辆行驶轨迹
for i = 1:20:length(xmr)
    Car=[xa(i),xb(i),xc(i),xd(i),xa(i);
        ya(i),yb(i),yc(i),yd(i),ya(i)];%车子的相对坐标
    plot(Car(1,:),Car(2,:),'color',[0.62,0.62,0.98],'linewidth',1)
    hold on 
end
% 
% %车位
Car=[0,3.2,3.2,5.8,5.8,14;
       4,4,0,0,4,4];%车子的相对坐标
plot(Car(1,:),Car(2,:),'-r','LineWidth',2)
% %范围、线宽、图底颜色
% box off
axis([-1 14 -1 7]); 
% 
% %字体大小
% set(gca,'FontSize',24);
% 
%坐标轴粗细
set(gca,'LineWidth',2)
% 
%轴的名称
xlabel('X(m)');% x轴名称
ylabel('Y(m)'); 
% 
% %带箭头的XY轴
annotation('arrow',[0.13 0.13],[0.06 0.99],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);
annotation('arrow',[0.1 0.95],[0.126 0.126],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);
% 
% %修改XY轴的刻度
% set(gca,'XTickLabel',{0:2:14})
% set(gca,'YTickLabel',{'','0','1','2','3','4','5','6','7'})
% 
text(9.5,9.5,'{\itM_1}','FontSize',20)
text(4.5,4.5,'{\itM_2}','FontSize',20)
text(4.5,0.8,'{\itM_3}','FontSize',20)
scatter(9.5,9.5,'k','filled')
scatter(4.5,4.5,'k','filled')
scatter(4.5,0.8,'k','filled')
%添加曲线注释
legend({'a点轨迹','b点轨迹','c点轨迹','d点轨迹','后轴中心轨迹'},'FontSize',24,'Location','best')

legend('boxoff')

%%
%路径曲率
figure
plot(xmr,tho,'k','LineWidth',2)

%上下限
hold on
plot([0,9.9],[1/Rmin,1/Rmin],'--r','LineWidth',2)
plot([0,9.9],[-1/Rmin,-1/Rmin],'--r','LineWidth',2)

box off
set(gca,'FontSize',40);
%坐标轴范围
axis([0 10 -0.3 0.3]); 
%坐标轴粗细
set(gca,'LineWidth',2)
%轴的名称
xlabel('X(m)');% x轴名称
ylabel('路径曲率(m^{-1})'); 

%修改XY轴的刻度
set(gca,'XTick',0:2:10)
set(gca,'XTickLabel',{0:2:10})
set(gca,'YTick',-0.3:0.1:0.3)
set(gca,'YTickLabel',{-0.3:0.1:0.3})

%%
%等效前轮转角
figure
plot(xmr,delta_fr,'k','LineWidth',2)

%上下限
hold on
plot([0,9.9],[0.524,0.524],'--r','LineWidth',2)
plot([0,9.9],[-0.524,-0.524],'--r','LineWidth',2)

box off
set(gca,'FontSize',40);
%坐标轴范围
axis([0 10 -0.6 0.6]); 
%坐标轴粗细
set(gca,'LineWidth',2)
%轴的名称
xlabel('X(m)');% x轴名称
ylabel('等效前轮转角(rad)'); 
%修改XY轴的刻度
set(gca,'XTick',0:2:10)
set(gca,'XTickLabel',{0:2:10})
set(gca,'YTick',-0.6:0.2:0.6)
set(gca,'YTickLabel',{-0.6:0.2:0.6})


%%
%等效前轮转角转速
figure
plot(xmr,omiga_fr,'k','LineWidth',2)

%上下限
hold on
plot([0,9.9],[0.524,0.524],'--r','LineWidth',2)

box off
set(gca,'FontSize',40);
%坐标轴范围
axis([0 10 -0.4 0.6]); 
%坐标轴粗细
set(gca,'LineWidth',2)
%轴的名称
xlabel('X(m)');% x轴名称
ylabel('等效前轮转角转速(rad/s)'); 
%修改XY轴的刻度
set(gca,'XTick',0:2:10)
set(gca,'XTickLabel',{0:2:10})
set(gca,'YTick',-0.4:0.2:0.6)
set(gca,'YTickLabel',{-0.4:0.2:0.6})


