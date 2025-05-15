%% �����ײ���չʾ
function step_num_random = random_display(Tag, node_num)
%�����˴�����ԭ�㿪ʼ��ɨ�������˲����м��书��
%Ĭ����ɨ���������ң����ϣ���������
%������д���Ϊ������������滮·�������д���
step_num_random = 0;
sweep_order = [1 0;0 1;-1 0;0 -1];%�ֱ��ʾ���ң����ϣ���������
figure(2)
[xData, yData] = netplot(Tag);
sign = Tag;
%δ��ɨ������Ϊ��ɫ����ɨ��һ�ε�����Ϊ��ɫ���ظ���ɨ��������Ϊ��ɫ
[map_x, map_y] = size(Tag);
xx = [0 1 1 0];
yy = [0 0 1 1];
patch('xData', xx, 'yData', yy, 'FaceColor', 'b');
pause_time = 0.01;
pause(pause_time)
sign(1, 1) = 1;
index = [1 1];
% for i = 1:max_step
step_num_random = 1;
count = 1;
while count < node_num
    for j = 1:4
        time = ceil(rand * 4);
        dir = sweep_order(time, :);
        if index(1)+dir(1)<1 || index(1)+dir(1)>map_x ||...
                index(2)+dir(2)<1 || index(2)+dir(2)>map_y
            continue
        end
        if Tag(index(1)+dir(1), index(2)+dir(2)) == 1
            continue
        end
        index = index + dir;
        x_temp = [0 1 1 0] +index(1) - 1;
        y_temp = [0 0 1 1] + index(2) - 1;
        if sign(index(1), index(2)) == 0
            patch('xData', x_temp, 'yData', y_temp, 'FaceColor', 'b');
            pause(pause_time)
            sign(index(1), index(2)) = 1;
            step_num_random = step_num_random + 1;
            count = count + 1;
            break
        else
            patch('xData', x_temp, 'yData', y_temp, 'FaceColor', 'r');
            pause(pause_time)
            step_num_random = step_num_random + 1;
            break
        end
    end
end