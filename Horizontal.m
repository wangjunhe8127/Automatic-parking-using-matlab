%% 
%���е�ʱ����Էֽ����У�����˳�������У����ȫ������Ҳ�У��������к󵯳��Ļ�ͼ����̫��
%%
    clc
    clear
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
    %��ֹ������
    xM4 = 0.8;
    yM4 = 1.1;    
    %��ʼ������
    xM0 = 9;
    yM0 = 3.8;    
    
    % ��ʼ������ת��������ֹ��Ϊ����ԭ��
    xM1 = xM0-xM4;
    yM1 = yM0-yM4;
    
    %����a b c��ֵ
    if xM0>(9.8-((4.3-yM0)*(9.8-8.4)/(4.3-3.3)))
        a0 = yM1/(9.8-((4.3-yM0)*(9.8-8.2)/(4.3-3.3))-xM4);
        b0 = -yM1/(2*pi);
        c0 = 2*pi/(9.8-((4.3-yM0)*(9.8-8.2)/(4.3-3.3))-xM4);      
    else 
        a0 = yM1/xM1;
        b0 = -yM1/(2*pi);
        c0 = 2*pi/xM1;              
    end
    
    %���ȶ����������
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
        %�ж�x��������Χ
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
        %������������   
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
% ��ͼ
% ��������·��ͼ
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


%������ʻ�켣
for i = 1:20:length(xmr)
    Car=[xa(i,1),xb(i,1),xc(i,1),xd(i,1),xa(i,1);ya(i,1),yb(i,1),yc(i,1),yd(i,1),ya(i,1)];%���ӵ��������

    plot(Car(1,:),Car(2,:),'color',[0.62,0.62,0.98],'linewidth',1)
    hold on 
end

Car02=[xa(length(xmr),1),xb(length(xmr),1),xc(length(xmr),1),xd(length(xmr),1),xa(length(xmr),1);...
       ya(length(xmr),1),yb(length(xmr),1),yc(length(xmr),1),yd(length(xmr),1),ya(length(xmr),1)];%���ӵ��������

plot(Car02(1,:),Car02(2,:),'color',[0.62,0.62,0.98],'linewidth',1)

plot(xa,ya,'color',color01,'linewidth',2)
plot(xb,yb,'color',color02,'linewidth',2)
plot(xc,yc,'color',color03,'linewidth',2)
plot(xd,yd,'color',color04,'linewidth',2)
%plot(xmr,ymr,'color',color05,'linewidth',2)

%��λ
Car=[-1,0,0,7,7,14;2.2,2.2,0,0,2.2,2.2];%���ӵ��������
plot(Car(1,:),Car(2,:),'-r','LineWidth',2)

%�߽���
plot([-1,13.9],[6.2,6.2],'--r','LineWidth',2)

%��Χ���߿�ͼ����ɫ
box off
axis([-1 14 -1 7]); 

%�����С
set(gca,'FontSize',24);

%�������ϸ
set(gca,'LineWidth',2)

%�������
xlabel('X(m)');% x������
ylabel('Y(m)'); 

%����ͷ��XY��
annotation('arrow',[0.13 0.13],[0.06 0.99],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);
annotation('arrow',[0.1 0.95],[0.126 0.126],'LineWidth',2,'HeadStyle','plain','HeadLength',18,'HeadWidth',8);

%�޸�XY��Ŀ̶�
set(gca,'XTickLabel',{0:2:14})
set(gca,'YTickLabel',{'','0','1','2','3','4','5','6','7'})

text(9,3.5,'{\itM_1}','FontSize',20)
text(0.45,1,'{\itM_2}','FontSize',20)

%�������ע��
legend({'a��켣','b��켣','c��켣','d��켣','�������Ĺ켣'},'FontSize',24,'Location','best')

%legend('boxoff')

%%
%·������
%�ֱ�
%��ڽ�
figure
plot(xmr,phi,'k','LineWidth',2)

box off
set(gca,'FontSize',40);
%�����᷶Χ
axis([0 10 0 0.6]); 
%�������ϸ
set(gca,'LineWidth',2)
%�������
xlabel('X(m)');% x������
ylabel('��ڽ�(rad)'); 
%�޸�XY��Ŀ̶�
set(gca,'XTick',0:2:10)
set(gca,'XTickLabel',{0:2:10})
set(gca,'YTick',0:0.1:0.6)
set(gca,'YTickLabel',{0:0.1:0.6})


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

