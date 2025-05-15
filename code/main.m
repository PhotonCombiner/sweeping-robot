clc
clear
close all
% 返回深度优先搜索实现全覆盖的运行次数
%将栅格模型的每一个栅格看成一个点
%实际中栅格模型是连续的，在计算机处理时看作离散的
%将栅格模型抽象为标识矩阵，矩阵对应位置的标记表示栅格对应位置的状态
%0表示对应位置无障碍物，1表示对应位置是障碍物
%参数设置
%% 定义变量
% axisStart = [0 0 0];
% axisLWH = [30 30 10];
% 
% %定义障碍物
% cubeInfo.exist = 0;
% cubeInfo = createCubeObject(cubeInfo);       %创建长方体障碍物信息
size1 = 1; %扫地机器人的尺寸
map_x = 30; %房间的尺寸
map_y = 30;
Xdestination = 24.5;%设置充电地点
Ydestination = 29.5;
A_temp = [Xdestination-0.5;Xdestination-0.5;Xdestination+0.5;Xdestination+0.5];%将路径终点设置为深绿色
B_temp = [Ydestination-0.5;Ydestination+0.5;Ydestination+0.5;Ydestination-0.5];
save datA A_temp
save datB B_temp
%{
%生成地图网格对应的标识矩阵
% tag = zeros(map_x, map_y);
%随机生成障碍物，也是通过矩阵生成
% G = barrier_generate(tag);
% G(1,1) = 0;
% save('D:\Desktop\saodi\map30.mat','G');%将生成的随机地图以mat格式保存到本地，这个路径需改为自己电脑的路径和其他代码放一起
%}
%% 调用地图
 load map0.mat
 P = rot90(G,3);%将覆盖算法识别矩阵逆时针旋转270，以保证和蚁群构造同一地图
%% 蚁群
h=rot90(abs(peaks(30)));
global MM
global Dir 
global Lgrid 
%Lgrid = input('请输入栅格粒径：');
Lgrid = 1;
MM=size(G,1);  %MM为矩阵维数
% figure(1);
for i=1:MM
  for j=1:MM
  x1=(j-1)*Lgrid;y1=(MM-i)*Lgrid; 
  x2=j*Lgrid;y2=(MM-i)*Lgrid; 
  x3=j*Lgrid;y3=(MM-i+1)*Lgrid; 
  x4=(j-1)*Lgrid;y4=(MM-i+1)*Lgrid; 
  %f=(max(max(h))-h(i,j))/max(max(h));
    if G(i,j)==1 
        fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.2,0.2,0.2]); hold on %栅格为1，填充为黑色
    else 
        fill([x1,x2,x3,x4],[y1,y2,y3,y4],[1,1,1]); hold on %栅格为0，填充为白色
    end 
  end 
end
axis([0,MM*Lgrid,0,MM*Lgrid]) 
grid on
%}
%% 扫地
%构建图的邻接压缩表
 b = graph_convert(P);
 [re, node_num] = DFS(b, P);
 step_num_plan = length(re);
 result_display(re, P);%展示扫地函数
%进行随机碰撞过程展示
%step_num_random = random_display(Tag, node_num);%出第二个图
%% 初始化地图信息
%{
Xinitial = input('请输入初始点的X坐标：');
Yinitial = input('请输入初始点的Y坐标：');
%}
load datx x0
load daty y0
Xinitial = x0;
Yinitial = y0;
[initial ij_initial]= modify(Xinitial,Yinitial);
if max(ij_initial)>MM||G(ij_initial(1),ij_initial(2))==1
    error('初始点不能设在障碍物上或超出范围');
end
%{
Xdestination = input('请输入目标点的X坐标：');
Ydestination = input('请输入目标点的Y坐标：');
%}
Xdestination = 24.5;%设置充电地点
Ydestination = 29.5;
[destination ij_destination]= modify(Xdestination,Ydestination);
if max(ij_destination)>MM||G(ij_destination(1),ij_destination(2))==1
    error('目标点不能设在障碍物上或超出范围');
end
%% 计算距离启发矩阵dis
dis = zeros(MM,MM);
for i=1:MM
  for j=1:MM
   x = (j-0.5)*Lgrid;
   y = (MM-i+0.5)*Lgrid;
   dis(i,j) = sqrt(sum(([x y]-destination).^2));
  end
end
%% 计算距离转移矩阵D
D=zeros(MM^2,8);   %行号表示栅格标号，列号表示邻接的8个方向的栅格号
Dir = [-MM-1,-1,MM-1,MM,MM+1,1,1-MM,-MM];
 for i = 1:MM^2     %8方向转移距离矩阵初步构建
     Dirn = Dir+i;
     if G(i)==1
             D(i,:)=inf;
             continue
     end
         for j = 1:8
             if  Dirn(j)<=0||Dirn(j)>MM^2        %出界的情况，暂且为0
                 continue 
             end
             if G(Dirn(j))==1
                 D(i,j) = inf;
             elseif mod(j,2)==0         %偶数方向为上下左右方向
                 D(i,j) = 1;
             elseif j==1 %左上方向的情况，保证路线不会擦障碍物边沿走过
                 if (G(Dirn(2))+G(Dirn(8))==0)
                   D(i,j) = 1.4; 
                 else
                   D(i,j) = inf;   
                 end
             elseif (Dirn(j-1)<=0||Dirn(j-1)>MM^2)||(Dirn(j+1)<=0||Dirn(j+1)>MM^2)%排除掉垂直方向的栅格出界的情况
                 continue
             elseif G(Dirn(j-1))+G(Dirn(j+1))==0    %其余三个斜方向
                 D(i,j) = 1.4;
             else
                 D(i,j) = inf;
             end
         end
     
 end
%% 创造边界
 num = 1:MM^2;
 obs_up = find(mod(num,MM)==1);
 obs_up = obs_up(2:end-1);
 D(obs_up,[1,2,3])=inf;
 obs_down = find(mod(num,MM)==0);
 obs_down = obs_down(2:end-1);
 D(obs_down,[5,6,7])=inf;
 D(2:MM-1,[1,7,8]) = inf;
 D(MM^2-MM+2:MM^2-1,[3,4,5])=inf;
 D(1,[1,2,3,7,8])=inf;
 D(MM,[1,5,6,7,8])=inf;
 D(MM^2-MM+1,[1,2,3,4,5])=inf;
 D(MM^2,[3,4,5,6,7])=inf;
%% 参数初始化
tic
NC_max=30; m=35; t=8; Rho=0.1; Q=100; Omega=10; Mu=2; Lambda=2;
%% 绘制找到的最优路径
[R_best,F_best,L_best,T_best,L_ave,Shortest_Route,Shortest_Length]=standard(D,initial,destination,dis,h,NC_max,m,t,Rho,Omega,Mu,Lambda,Q); %函数调用
%绘制找到的最优路径
j = ceil(Shortest_Route/MM);
i = mod(Shortest_Route,MM);
i(i==0) = MM;
x = (j-0.5)*Lgrid;
y = (MM-i+0.5)*Lgrid;
x = [initial(1) x destination(1)];
y = [initial(2) y destination(2)];
figure(1);
plot(x,y,'-g','LineWidth',2);
xlabel('x'); ylabel('y'); title('最佳路径');
% colorMatCube = [0.2 0.2 0.2];
% pellucidity = 0.5;    %透明度
% hold on;
% drawCubeObject(cubeInfo,colorMatCube,pellucidity);     %画长方体障碍物
% view(3)
% grid on;
% axis equal;
% axis([0 30 0 30 0 10])
% xlabel('x')
% ylabel('y')
% zlabel('z')
grid on 
toc  %计算运行时间
%% 绘制收敛曲线
%{
figure(2); iter=1:length(L_best);
plot(iter,L_best,'-b','LineWidth', 1)
xlabel('迭代次数'); ylabel('各代最佳路线的长度'); 
axis([0,NC_max,25,90]);
grid on;hold on
figure(3); iter=1:length(L_best);
plot(iter,F_best*100,'-b','LineWidth',1)
xlabel('迭代次数'); ylabel('各代最佳路线的高度均方差*100'); 
axis([0,NC_max,0,30]);
grid on;hold on
figure(4); iter=1:length(L_best);
plot(iter,T_best,'-b','LineWidth',1)
xlabel('迭代次数'); ylabel('各代最佳路线的转弯次数'); 
axis([0,NC_max,5,50]);
grid on;hold on
toc
%}
