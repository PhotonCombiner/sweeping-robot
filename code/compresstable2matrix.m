% count
%% ͼ��ѹ���������ڽӾ����ת������DFS���Ӻ���
function A = compresstable2matrix(b, Tag)
%label�Ǻ�Tagͬά�ȵľ��󣬿��Դﵽ��λ����0�����ܴﵽ��λ����1
[n, ~] = size(b);
[map_x, map_y] = size(Tag);
m = max(b(:));
A = zeros(m,m);
for i = 1:n
    A(b(i,1),b(i,2)) = 1;
    A(b(i,2),b(i,1)) = 1;
end