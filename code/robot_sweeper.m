%% ɨ�ػ�����ģ�⣬������
 function [step_num_plan, step_num_random] = robot_sweeper()
% ���������������ʵ��ȫ���ǵ����д���
%��դ��ģ�͵�ÿһ��դ�񿴳�һ����
%ʵ����դ��ģ���������ģ��ڼ��������ʱ������ɢ��

%��դ��ģ�ͳ���Ϊ��ʶ���󣬾����Ӧλ�õı�Ǳ�ʾդ���Ӧλ�õ�״̬
%0��ʾ��Ӧλ�����ϰ��1��ʾ��Ӧλ�����ϰ���

%��������
size1 = 1; %ɨ�ػ����˵ĳߴ�
map_x = 30%20; %����ĳߴ�
map_y = 30%15;
%{%���ɵ�ͼ�����Ӧ�ı�ʶ����
tag = zeros(map_x, map_y);
%��������ϰ��Ҳ��ͨ����������
Tag = barrier_generate(tag);
Tag(1, 1) = 0;

%save('D:\Desktop\Robot-Sweeper-Path-Programming\Robot-Sweeper-Path-Programming\code\map.mat','Tag');%�����ɵ������ͼ��mat��ʽ���浽���أ����·�����Ϊ�Լ����Ե�·�������������һ��
%}
%load map0.mat%���ص�ͼ
%����ͼ���ڽ�ѹ����
 b = graph_convert(Tag);
 [re, node_num] = DFS(b, Tag);
 step_num_plan = length(re);
 result_display(re, Tag);%չʾɨ�غ���
%���������ײ����չʾ
%step_num_random = random_display(Tag, node_num);%���ڶ���ͼ






