%% 绘制地图以及障碍物的位置
function [xData, yData] = netplot(Tag)
[map_x, map_y] = size(Tag);
x = 0:map_x;
y = 0:map_y;
x1 = [x(1) x(end)]';
y1 = [y(1) y(end)]';
%产生所有网格线的数据xData,yData
x2 = repmat(x1, 1, length(y)-2);
x3 = repmat(x(2) : x(end-1), 2, 1);
xData = [x2 x3];
y2 = repmat(y1, 1, length(x)-2);
y3 = repmat(y(2):y(end-1), 2, 1);
yData = [y3 y2];
h = line(xData, yData);
box on;
set(h, 'Color', 'k');
%绘出障碍物的位置，用黑色区域表示
[co_x, co_y] = find(Tag == 1);
% [co_x co_y]
for i = 1:length(co_x)
    x = [0 1 1 0] + co_x(i) -1;
    y = [0 0 1 1] + co_y(i) -1;
    patch('xData', x, 'yData', y, 'FaceColor', 'k');
end
%save('D:\Desktop\Robot-Sweeper-Path-Programming\Robot-Sweeper-Path-Programming\code\map.mat','Tag');%将生成的随机地图以mat格式保存到本地，便于和标准算法在同一个地图下进行对比，这个路径可以改为自己电脑的路径
%load map.mat%下载地图
%% 定义变量
axisStart = [0 0 0];
axisLWH = [30 30 10];

%定义障碍物
cubeInfo.exist = 0;
cubeInfo = createCubeObject(cubeInfo);       %创建长方体障碍物信息
colorMatCube = [0.2 0.2 0.2];
pellucidity = 0.8;    %透明度
hold on;
drawCubeObject(cubeInfo,colorMatCube,pellucidity);     %画长方体障碍物
view(3)
view(-60,45)
grid on;
axis equal;
axis([0 30 0 30 0 10])
xlabel('x')
ylabel('y')
zlabel('z')