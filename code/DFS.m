%% �����������
function [re, node_num] = DFS(b, Tag)
%����ͼ��ѹ���������������������·��
%ѹ���������ֵ�����ڽӾ���Ŀ����
m = max(b(:));
%���ڽ�ѹ������ͼ�ľ����ʾ
A = compresstable2matrix(b, Tag);
node_num = 0;
for i = 1:length(A)
    if isempty(find(A(i,:) == 1)) ~= 1
        node_num = node_num + 1;
    end
end
count = 1;
top = 1;
stack(top) = 1; %����һ���ڵ���ջ
flag = 1; %���ĳ���ڵ��Ƿ���ʹ���
re = [];
% node_num
while top ~= 0 %�ж϶�ջ�Ƿ�Ϊ��
    pre_len = length(stack); %��Ѱ��һ���ڵ�ǰ�Ķ�ջ����
    i = stack(top); %ȡ��ջ���ڵ�
    for j = 1:m
        if A(i,j) == 1 && isempty(find(flag == j,1)) %����ڵ���������û�з��ʹ�
            top = top+1; %��չ��ջ
            stack(top) = j; %�½ڵ���ջ
            flag = [flag j]; %���½ڵ���б��
            re = [re;i j]; %���ߴ�����
            count = count + 1;
            break
        end
    end
    %�����ջ����û�����ӣ���ڵ㿪ʼ��ջ
    if length(stack) == pre_len
        if count == node_num
            break
        end
        if top ~= 1
            re = [re;stack(top) stack(top-1)];
            stack(top) = [];
            top = top-1;
        else
            %             re = [re;stack(top) 1];
            stack(top) = [];
            top = top-1;
        end
    end
end