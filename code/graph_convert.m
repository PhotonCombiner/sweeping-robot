%% ����Ǿ���ת��Ϊͼ��ѹ������
function b = graph_convert(Tag)
[map_x, map_y] = size(Tag);
b = [];
%����ÿ��ÿ���ҵ���Ԫ������λ�ã������ڵķ���Ԫ��������
for i = 1:map_x
    index = find(Tag(i,:) == 0);
    for j = 1:length(index) - 1
        if ((index(j+1) - index(j) == 1))
            % i��ʾ�����ڵ�������index(j)��ʾ�����ڵ�����
            b = [b;[index(j)+(i-1)*map_y index(j+1)+(i-1)*map_y]];
        else
            continue
        end
    end
end
for i = 1:map_y
    index = find(Tag(:,i) == 0);
    for j = 1:length(index) - 1
        if (index(j+1) - index(j) == 1)
            % i��ʾ�����ڵ�������index(j)��ʾ�����ڵ�����
            b = [b;[i+(index(j)-1)*map_y i+(index(j+1)-1)*map_y]];
        else 
            continue
        end
    end
end
b = sortrows(b, 1);
