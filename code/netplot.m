%% ���Ƶ�ͼ�Լ��ϰ����λ��
function [xData, yData] = netplot(Tag)
[map_x, map_y] = size(Tag);
x = 0:map_x;
y = 0:map_y;
x1 = [x(1) x(end)]';
y1 = [y(1) y(end)]';
%�������������ߵ�����xData,yData
x2 = repmat(x1, 1, length(y)-2);
x3 = repmat(x(2) : x(end-1), 2, 1);
xData = [x2 x3];
y2 = repmat(y1, 1, length(x)-2);
y3 = repmat(y(2):y(end-1), 2, 1);
yData = [y3 y2];
h = line(xData, yData);
box on;
set(h, 'Color', 'k');
%����ϰ����λ�ã��ú�ɫ�����ʾ
[co_x, co_y] = find(Tag == 1);
% [co_x co_y]
for i = 1:length(co_x)
    x = [0 1 1 0] + co_x(i) -1;
    y = [0 0 1 1] + co_y(i) -1;
    patch('xData', x, 'yData', y, 'FaceColor', 'k');
end
%save('D:\Desktop\Robot-Sweeper-Path-Programming\Robot-Sweeper-Path-Programming\code\map.mat','Tag');%�����ɵ������ͼ��mat��ʽ���浽���أ����ںͱ�׼�㷨��ͬһ����ͼ�½��жԱȣ����·�����Ը�Ϊ�Լ����Ե�·��
%load map.mat%���ص�ͼ
%% �������
axisStart = [0 0 0];
axisLWH = [30 30 10];

%�����ϰ���
cubeInfo.exist = 0;
cubeInfo = createCubeObject(cubeInfo);       %�����������ϰ�����Ϣ
colorMatCube = [0.2 0.2 0.2];
pellucidity = 0.8;    %͸����
hold on;
drawCubeObject(cubeInfo,colorMatCube,pellucidity);     %���������ϰ���
view(3)
view(-60,45)
grid on;
axis equal;
axis([0 30 0 30 0 10])
xlabel('x')
ylabel('y')
zlabel('z')