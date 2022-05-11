%% 
%运行的时候可以分节运行，按照顺序挨着运行，如果全部运行也行，但是运行后弹出的绘图窗口太多
%%
    clc
    clear
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
    %终止点坐标
    xM4 = 0.8;
    yM4 = 1.1;    
    %起始点坐标
    xM0 = 9;
    yM0 = 3.8;    
    
    % 起始点坐标转换，以终止点为坐标原点
    xM1 = xM0-xM4;
    yM1 = yM0-yM4;
    
    %计算a b c的值
    if xM0>(9.8-((4.3-yM0)*(9.8-8.4)/(4.3-3.3)))
        a0 = yM1/(9.8-((4.3-yM0)*(9.8-8.2)/(4.3-3.3))-xM4);
        b0 = -yM1/(2*pi);
        c0 = 2*pi/(9.8-((4.3-yM0)*(9.8-8.2)/(4.3-3.3))-xM4);      
    else 
        a0 = yM1/xM1;
        b0 = -yM1/(2*pi);
        c0 = 2*pi/xM1;              
    end
    
    %首先定义各个变量
    k = 1;
    omiga_fr = [];
    tho = [];
    ya = [];
    yb = [];
    yc = [];
    yd = [];
    xa = [];
    xb = [];
    xc = [];
    xd = [];
    
    for xm = 0:0.01:xM1
        xmr(k,1) = xm+xM4;
        ym(k,1) = a0*xm+b0*sin(c0*xm);
        ymr(k,1) = ym(k,1)+yM4;
        phi(k,1) = atan(a0+b0*c0*cos(c0*xm));
        d_ym(k,1) = a0+b0*c0*cos(c0*xm);
        d2_ym(k,1) = -b0*c0*c0*sin(c0*xm);
        %判断x所在区域范围
        if xmr(k,1) >= (9.8-((4.3-yM0)*(9.8-8.2)/(4.3-3.3)))
            ymr(k,1) = yM0;
            phi(k,1) = 0;  
            d_ym(k,1) = 0;
            d2_ym(k,1) = 0;
        end    
        
        tho(k,1) = d2_ym(k,1)/((1+d_ym(k,1)^2)^(3/2));
        
        delta_fr(k,1) = atan(2.48*(d2_ym(k,1))/((1+d_ym(k,1)^2)^(3/2)));
        
%         omiga_fr(k,1) = (1/(1+(d_ym(k,1))^2)^(1/2))*diff(delta_fr(k,1),xm);
        
        omiga_fr(k,1) = -((62*b0*c0^3*cos(c0*xm))/(25*((a0 + b0*c0*cos(c0*xm))^2 + 1)^(3/2)) + (186*b0^2*c0^4*sin(c0*xm)^2*(a0 + b0*c0*cos(c0*xm)))/(25*((a0 + b0*c0*cos(c0*xm))^2 + 1)^(5/2)))/(((a0 + b0*c0*cos(c0*xm))^2 + 1)^(1/2)*((3844*b0^2*c0^4*sin(c0*xm)^2)/(625*((a0 + b0*c0*cos(c0*xm))^2 + 1)^3) + 1));
        if xmr(k,1) >= (9.8-((4.3-yM0)*(9.8-8.2)/(4.3-3.3)))
            omiga_fr(k,1) = 0;
        end    
        %车辆各点坐标   
        xa(k,1)=xmr(k,1)-lr*cos(phi(k,1))+W/2*sin(phi(k,1));
        xb(k,1)=xmr(k,1)+l*cos(phi(k,1))+(L-l-lr)*cos(phi(k,1))+W/2*sin(phi(k,1));
        xc(k,1)=xmr(k,1)+l*cos(phi(k,1))+(L-l-lr)*cos(phi(k,1))-W/2*sin(phi(k,1));
        xd(k,1)=xmr(k,1)-lr*cos(phi(k,1))-W/2*sin(phi(k,1));
        
        ya(k,1)=ymr(k,1)-lr*sin(phi(k,1))-W/2*cos(phi(k,1));
        yb(k,1)=ymr(k,1)+(L-lr)*sin(phi(k,1))-W/2*cos(phi(k,1));
        yc(k,1)=ymr(k,1)+(L-lr)*sin(phi(k,1))+W/2*cos(phi(k,1));
        yd(k,1)=ymr(k,1)-lr*sin(phi(k,1))+W/2*cos(phi(k,1));
        
        k = k+1;
    end
%%
% 绘图
% 车辆各点路径图
figure
box off
set(0,'defaultfigurecolor','w')
hold on

color01 = [1,0,1];
color02 = [0,1,0];
color03 = [1,1,0];
color04 = [0,1,1];
color05 = [0,0,1];

plot(xa,ya,'color',color01,'linewidth',2)
plot(xb,yb,'color',color02,'linewidth',2)
plot(xc,yc,'color',color03,'linewidth',2)
plot(xd,yd,'color',color04,'linewidth',2)
plot(xmr,ymr,'color',color05,'linewidth',2)


%车辆行驶轨迹
for i = 1:20:length(xmr)
    Car=[xa(i,1),xb(i,1),xc(i,1),xd(i,1),xa(i,1);ya(i,1),yb(i,1),yc(i,1),yd(i,1),ya(i,1)];%车子的相对坐标

    plot(Car(1,:),Car(2,:),'color',[0.62,0.62,0.98],'linewidth',1)
    hold on 
end

Car02=[xa(length(xmr),1),xb(length(xmr),1),xc(length(xmr),1),xd(length(xmr),1),xa(length(xmr),1);...
       ya(length(xmr),1),yb(length(xmr),1),yc(length(xmr),1),yd(length(xmr),1),ya(length(xmr),1)];%车子的相对坐标

plot(Car02(1,:),Car02(2,:),'color',[0.62,0.62,0.98],'linewidth',1)

plot(xa,ya,'color',color01,'linewidth',2)
plot(xb,yb,'color',color02,'linewidth',2)
plot(xc,yc,'color',color03,'linewidth',2)
plot(xd,yd,'color',color04,'linewidth',2)
%plot(xmr,ymr,'color',color05,'linewidth',2)

%车位
Car=[-1,0,0,7,7,14;2.2,2.2,0,0,2.2,2.2];%车子的相对坐标
plot(Car(1,:),Car(2,:),'-r','LineWidth',2)

%边界线
plot([-1,13.9],[6.2,6.2],'--r','LineWidth',2)

%范围、线宽、图底颜色
box off
axis([-1 14 -1 7]); 

%字体大小
set(gca,'FontSize',24);

%坐标轴粗细
set(gca,'LineWidth',2)

%轴的名称
xlabel('X(m)');% x轴名称
ylabel('Y(m)'); 

%带箭头的XY轴
annotation('arrow',[0.13 0.13],[0.06 0.99],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);
annotation('arrow',[0.1 0.95],[0.126 0.126],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);

%修改XY轴的刻度
set(gca,'XTickLabel',{0:2:14})
set(gca,'YTickLabel',{'','0','1','2','3','4','5','6','7'})

text(9,3.5,'{\itM_1}','FontSize',20)
text(0.45,1,'{\itM_2}','FontSize',20)

%添加曲线注释
legend({'a点轨迹','b点轨迹','c点轨迹','d点轨迹','后轴中心轨迹'},'FontSize',24,'Location','best')

%legend('boxoff')

%%
%路径参数
%分别画
%横摆角
figure
plot(xmr,phi,'k','LineWidth',2)

box off
set(gca,'FontSize',40);
%坐标轴范围
axis([0 10 0 0.6]); 
%坐标轴粗细
set(gca,'LineWidth',2)
%轴的名称
xlabel('X(m)');% x轴名称
ylabel('横摆角(rad)'); 
%修改XY轴的刻度
set(gca,'XTick',0:2:10)
set(gca,'XTickLabel',{0:2:10})
set(gca,'YTick',0:0.1:0.6)
set(gca,'YTickLabel',{0:0.1:0.6})


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

