%% 
%���е�ʱ����Էֽ����У�����˳�������У����ȫ������Ҳ�У��������к󵯳��Ļ�ͼ����̫��
%         omiga_fr(k,1) = (1/(1+(d_ym(k,1))^2)^(1/2))*diff(delta_fr(k,1),xm);
%%
clc
clear
close
%��������   
L = 3.95;  %����/m
W = 1.97;  %����/m
l = 2.48;   %���/m
lf = 0.8;    %ǰ��/m
lr = 0.67;   %����/m
delta_f = 0.524;  %ǰ�����ת��/rad
omiga_f = 0.524;  %ǰ�����ת��ת��/rad/s 
Rmin = 4.3;  %��Сת��뾶/m
h = 4;      %��·���/m
Lp = 7;   %��λ����/m
Wp = 2.2;    %��λ���/m    
%��ʼ������
xM1 = 4.5;
yM1 = 0.8;
%�м������
xM2 = 4.5;
yM2 = 4.5;
%ĩ�˵�����
xM3 = 9.5;
yM3 = 9.5;     
%����
DX32 = xM3-xM2;
DY32 = yM3-yM2;
DY21 = yM2-yM1;    
%���ȶ����������
k = 1;
ya = [];yb = [];yc = [];yd = [];
xa = [];xb = [];xc = [];xd = [];
xmr = [];ym = [];ymr = [];
phi = [];d_ym = [];d2_ym = [];
tho = [];delta_fr = [];tmp = [];
for xm = 0:0.01:DY21
    xmr = [xmr;xM1];%�����ƶ�
    ymr = [ymr;yM1+xm];%�滮��y����
    phi = [phi;pi/2];%y����б�ʵ�arctan���Եõ��Ƕ�
    d_ym = [d_ym;inf];%y����б��
    d2_ym = [d2_ym;0];%y����б��
    tho = [tho;d2_ym(k)/((1+d_ym(k)^2)^(3/2))];
    delta_fr = [delta_fr;atan(l*(d2_ym(k))/((1+d_ym(k)^2)^(3/2)))];
    %������������   
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
%Բ������
x0 = 9.5;
y0 = 4.5;
R = 5;
for xm = 0:0.01:DX32
    xmr = [xmr;xM1+xm];%�����ƶ�
    ymr = [ymr;y0+(R^2-(xmr(k)-x0)^2)^(1/2)];%�滮��y����
    phi = [phi;atan(-(xmr(k)-x0)/(ymr(k)-y0))];%y����б�ʵ�arctan���Եõ��Ƕ�
    d_ym = [d_ym;-(xmr(k)-x0)/(ymr(k)-y0)];%y����б��
    d2_ym = [d2_ym;(-1*(ymr(k)-y0)+d_ym(k)*(xmr(k)-x0))/(ymr(k)-y0)^2];%yб�ʵĶ��η�
    tho = [tho;d2_ym(k)/((1+d_ym(k)^2)^(3/2))];%���ʹ�ʽ����ʾ�����̶�
    delta_fr = [delta_fr;atan(l*(d2_ym(k))/((1+d_ym(k)^2)^(3/2)))];%��Чǰ��ת�ǣ�������(ע���е���l�е���1)
    %������������   
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
% ��ͼ
% ��������·��ͼ
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


% ������ʻ�켣
for i = 1:20:length(xmr)
    Car=[xa(i),xb(i),xc(i),xd(i),xa(i);
        ya(i),yb(i),yc(i),yd(i),ya(i)];%���ӵ��������
    plot(Car(1,:),Car(2,:),'color',[0.62,0.62,0.98],'linewidth',1)
    hold on 
end
% 
% %��λ
Car=[0,3.2,3.2,5.8,5.8,14;
       4,4,0,0,4,4];%���ӵ��������
plot(Car(1,:),Car(2,:),'-r','LineWidth',2)
% %��Χ���߿�ͼ����ɫ
% box off
axis([-1 14 -1 7]); 
% 
% %�����С
% set(gca,'FontSize',24);
% 
%�������ϸ
set(gca,'LineWidth',2)
% 
%�������
xlabel('X(m)');% x������
ylabel('Y(m)'); 
% 
% %����ͷ��XY��
annotation('arrow',[0.13 0.13],[0.06 0.99],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);
annotation('arrow',[0.1 0.95],[0.126 0.126],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);
% 
% %�޸�XY��Ŀ̶�
% set(gca,'XTickLabel',{0:2:14})
% set(gca,'YTickLabel',{'','0','1','2','3','4','5','6','7'})
% 
text(9.5,9.5,'{\itM_1}','FontSize',20)
text(4.5,4.5,'{\itM_2}','FontSize',20)
text(4.5,0.8,'{\itM_3}','FontSize',20)
scatter(9.5,9.5,'k','filled')
scatter(4.5,4.5,'k','filled')
scatter(4.5,0.8,'k','filled')
%�������ע��
legend({'a��켣','b��켣','c��켣','d��켣','�������Ĺ켣'},'FontSize',24,'Location','best')

legend('boxoff')

%%
%·������
figure
plot(xmr,tho,'k','LineWidth',2)

%������
hold on
plot([0,9.9],[1/Rmin,1/Rmin],'--r','LineWidth',2)
plot([0,9.9],[-1/Rmin,-1/Rmin],'--r','LineWidth',2)

box off
set(gca,'FontSize',40);
%�����᷶Χ
axis([0 10 -0.3 0.3]); 
%�������ϸ
set(gca,'LineWidth',2)
%�������
xlabel('X(m)');% x������
ylabel('·������(m^{-1})'); 

%�޸�XY��Ŀ̶�
set(gca,'XTick',0:2:10)
set(gca,'XTickLabel',{0:2:10})
set(gca,'YTick',-0.3:0.1:0.3)
set(gca,'YTickLabel',{-0.3:0.1:0.3})

%%
%��Чǰ��ת��
figure
plot(xmr,delta_fr,'k','LineWidth',2)

%������
hold on
plot([0,9.9],[0.524,0.524],'--r','LineWidth',2)
plot([0,9.9],[-0.524,-0.524],'--r','LineWidth',2)

box off
set(gca,'FontSize',40);
%�����᷶Χ
axis([0 10 -0.6 0.6]); 
%�������ϸ
set(gca,'LineWidth',2)
%�������
xlabel('X(m)');% x������
ylabel('��Чǰ��ת��(rad)'); 
%�޸�XY��Ŀ̶�
set(gca,'XTick',0:2:10)
set(gca,'XTickLabel',{0:2:10})
set(gca,'YTick',-0.6:0.2:0.6)
set(gca,'YTickLabel',{-0.6:0.2:0.6})


%%
%��Чǰ��ת��ת��
figure
plot(xmr,omiga_fr,'k','LineWidth',2)

%������
hold on
plot([0,9.9],[0.524,0.524],'--r','LineWidth',2)

box off
set(gca,'FontSize',40);
%�����᷶Χ
axis([0 10 -0.4 0.6]); 
%�������ϸ
set(gca,'LineWidth',2)
%�������
xlabel('X(m)');% x������
ylabel('��Чǰ��ת��ת��(rad/s)'); 
%�޸�XY��Ŀ̶�
set(gca,'XTick',0:2:10)
set(gca,'XTickLabel',{0:2:10})
set(gca,'YTick',-0.4:0.2:0.6)
set(gca,'YTickLabel',{-0.4:0.2:0.6})


