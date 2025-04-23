% 방 크기 설정 (단위: m)
roomWidth  = 4;    % X 방향 길이
roomDepth  = 4;    % Y 방향 길이
roomHeight = 2.5;  % Z 방향 높이

% 방 모서리 좌표 정의
x = [0 roomWidth roomWidth 0 0];
y = [0 0 roomDepth roomDepth 0];
z_bottom = zeros(size(x));
z_top    = roomHeight * ones(size(x));

figure
hold on

% 바닥 테두리
plot3(x, y, z_bottom, 'k-', 'LineWidth', 1.5);

% 천장 테두리
plot3(x, y, z_top, 'k-', 'LineWidth', 1.5);

% 세로 모서리
for i = 1:4
    plot3([x(i) x(i)], [y(i) y(i)], [0 roomHeight], 'k-', 'LineWidth', 1.5);
end

% 보기 설정
axis equal        % 비율 고정
view(30, 20)      % 원하는 시점으로 조정

% 내부 그리드 및 축 테두리 끄기
grid off
box off

% 축 자체와 숫자/눈금 모두 숨기기
axis off

hold off