%% =====================================================================
%  AutoDrive HW#1 - Task #1-1 : 사각형(Rectangular) Trajectory
%  학번: 201912700
%
%  목표 : 모바일 로봇이 사각형 경로를 따라 한 바퀴 이동하는 모습을
%         시뮬레이션한다. 단, 로봇은 "항상 진행 방향을 바라보아야" 한다.
%
%  사용 도구 : Mobile Robotics Simulation Toolbox 의 Visualizer2D 객체
% =====================================================================

%% [0] 초기화 ---------------------------------------------------------
clear;       % Workspace(작업공간)에 남아있는 모든 변수 삭제
clc;         % Command Window(명령창) 출력 내용 지우기
close all;   % 열려 있는 모든 그림(figure) 창 닫기

%% [1] 시각화 객체 생성 ----------------------------------------------
viz = Visualizer2D;       % 2D 로봇 시각화 "도구(객체)" 하나 만들기
viz.hasWaypoints = true;  % 경유점(waypoint)을 화면에 표시하도록 켜기

%% [2] 사각형의 꼭짓점(corner) 정의 ---------------------------------
%  각 행은 [x, y] 좌표이며 단위는 meter(m) 이다.
%  마지막 행에 시작점(0,0)을 한 번 더 넣어서 "닫힌 사각형"을 만든다.
%  (사이즈는 자유 — 여기서는 가로 4m, 세로 3m 로 설정)
corners = [0, 0;     % ① 시작점 (좌하단)
           4, 0;     % ② 우하단
           4, 3;     % ③ 우상단
           0, 3;     % ④ 좌상단
           0, 0];    % ⑤ 다시 시작점으로 복귀

waypoints = corners(1:4, :);   % 화면에 표시할 꼭짓점 4개만 따로 보관

%% [3] 이동 설정 -----------------------------------------------------
stepSize = 0.1;   % 한 스텝(step)당 이동 거리 [m]. 작을수록 더 부드럽게 움직임.

% --- (GIF 녹화용 설정) -------------------------------------------
gifFile    = 'HW1_1.gif';  % 저장될 GIF 파일 이름 (작업 폴더에 생성)
firstFrame = true;                   % 첫 프레임인지 표시하는 깃발

%% [4] 사각형의 각 변(edge)을 차례대로 따라 이동 --------------------
for i = 1:size(corners,1)-1        % 변은 (꼭짓점 개수 - 1)개 있다
    startPt = corners(i,   :);     % 이번 변의 시작 꼭짓점 [x y]
    endPt   = corners(i+1, :);     % 이번 변의 끝   꼭짓점 [x y]

    delta  = endPt - startPt;              % 시작→끝 방향 벡터 [dx dy]
    segLen = norm(delta);                  % 이 변의 길이(직선거리)
    theta  = atan2(delta(2), delta(1));    % 진행 방향(heading) 각도 [rad]
                                           %  → 로봇이 바라볼 방향

    nSteps = round(segLen / stepSize);     % 이 변을 몇 번에 나눠 갈지

    for k = 0:nSteps
        ratio = k / nSteps;                % 0 → 1 로 변하는 진행 비율
        pos   = startPt + ratio * delta;   % 시작점과 끝점 사이의 현재 위치

        pose = [pos(1); pos(2); theta];    % 로봇의 자세 [x; y; theta]

        viz(pose, waypoints);   % 화면 갱신: 현재 위치/방향 + 경유점 표시
        axis([-1 5 -1 4]);      % (GIF 보기용) 축 범위 고정 → 화면 안 흔들림
        pause(0.02);            % 0.02초 멈춤 → 애니메이션처럼 보이게 함

        % --- (GIF 녹화) 현재 화면을 한 프레임씩 GIF에 추가 -------
        frame    = getframe(gcf);                 % 지금 그림 창을 한 컷 캡처
        [A, map] = rgb2ind(frame.cdata, 256);     % GIF용 256색 이미지로 변환
        if firstFrame
            imwrite(A, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.05);
            firstFrame = false;                   % 다음부터는 이어붙이기 모드
        else
            imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
        end
    end
end


%% =====================================================================
%  AutoDrive HW#1 - Task #1-2 : 원(Circular) Trajectory
%  학번: 201912700
%
%  목표 : 모바일 로봇이 원 경로를 따라 한 바퀴 돈다.
%         단, 로봇은 "항상 진행 방향(원의 접선 방향)을 바라보아야" 한다.
%
%  ※ 이 섹션만 따로 돌리려면: 이 줄 안에 커서를 두고  Ctrl + Enter
% =====================================================================

%% [0] 초기화 ---------------------------------------------------------
clear; clc; close all;

%% [1] 시각화 객체 생성 ----------------------------------------------
viz = Visualizer2D;        % 2D 로봇 시각화 객체 생성
% 원 운동에서는 경유점(waypoint)이 필요 없으므로 켜지 않는다.

%% [2] 원의 파라미터 정의 -------------------------------------------
center = [3, 3];   % 원의 중심 [x, y]  (단위: m)
R      = 2;        % 원의 반지름 [m]   (원반경은 자유롭게 바꿔도 됨)

%% [3] 이동 설정 -----------------------------------------------------
nSteps = 120;                       % 한 바퀴를 몇 조각으로 나눌지 (클수록 부드러움)
phiAll = linspace(0, 2*pi, nSteps); % 0 ~ 360도(2*pi rad)를 nSteps개로 균등 분할

% --- (GIF 녹화용 설정) -------------------------------------------
gifFile    = 'HW1_2.gif';           % 저장될 GIF 파일 이름 (작업 폴더에 생성)
firstFrame = true;                  % 첫 프레임인지 표시하는 깃발

%% [4] 원 둘레를 따라 이동 ------------------------------------------
for k = 1:nSteps
    phi = phiAll(k);                       % 현재 회전 각도 (원 위의 위치 각도)

    x = center(1) + R*cos(phi);            % 원 위의 현재 x 좌표
    y = center(2) + R*sin(phi);            % 원 위의 현재 y 좌표

    theta = phi + pi/2;                    % 진행 방향(접선) = 위치각 + 90도
                                           %  → 반시계 방향으로 돌 때의 헤딩

    pose = [x; y; theta];                  % 로봇 자세 [x; y; theta]

    viz(pose);                             % 화면 갱신 (경유점 없이 로봇만)
    axis([0 6 0 6]);                       % (GIF 보기용) 축 범위 고정 → 화면 안 흔들림
    pause(0.02);                           % 0.02초 멈춤 → 애니메이션 효과

    % --- (GIF 녹화) 현재 화면을 한 프레임씩 GIF에 추가 -----------
    frame    = getframe(gcf);                 % 지금 그림 창을 한 컷 캡처
    [A, map] = rgb2ind(frame.cdata, 256);     % GIF용 256색 이미지로 변환
    if firstFrame
        imwrite(A, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.05);
        firstFrame = false;                   % 다음부터는 이어붙이기 모드
    else
        imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end


%% =====================================================================
%  AutoDrive HW#1 - Task #3,4 : 네이버 지도 캡쳐하여 표시(팝업)하기
%  학번: 201912700
%
%  목표 : 네이버 지도를 캡쳐한 이미지(PNG)를 읽어서 Occupancy Grid(점유
%         격자)로 변환하고, 실행하면 지도 창이 팝업되도록 표시한다.
%
%  규칙 : 흰색 = 빈 공간(로봇이 다니는 길) / 검은색 = 장애물(건물 등, 못 감)
% =====================================================================

%% [0] 초기화 ---------------------------------------------------------
clear; clc; close all;

%% [1] 지도 이미지 불러오기 ------------------------------------------
mapFile = 'mymap_pknu.png';   % 작업 폴더에 저장한 지도 파일 이름
image   = imread(mapFile);    % 이미지를 읽어서 숫자 행렬로 가져오기

%% [2] 흑백(이진) 이미지로 변환 -------------------------------------
% 컬러 이미지면 회색조로 먼저 바꾼다. (이미 흑백이면 그대로 사용)
if size(image, 3) == 3        % 3번째 차원이 3이면 컬러(RGB)라는 뜻
    grayimage = rgb2gray(image);
else
    grayimage = image;
end

% 회색조 → 흑백. 기준값(threshold)보다 어두우면 '장애물(true)'로 본다.
%  ▶ 길이 흰색으로 잘 나오면 이대로 두면 됨.
%  ▶ 만약 흑백이 반대로(건물=길, 길=벽) 나오면 아래 줄을 이렇게 뒤집기:
%        bwimage = grayimage > 128;
bwimage = grayimage < 128;    % 128 = 0(검정)~255(흰색)의 중간값

%% [3] Occupancy Grid(점유 격자 지도) 만들기 -----------------------
map = binaryOccupancyMap(bwimage);   % 흑백 행렬을 '지도 객체'로 변환

%% [4] 지도 표시 (실행 시 창 팝업) ----------------------------------
figure;        % 새 그림 창 띄우기 (← 이게 있어야 팝업으로 뜸)
show(map);     % 지도를 그 창에 표시
title('My Map - PKNU Campus');   % 그림 제목


%% =====================================================================
%  AutoDrive HW#1 - Task #5 : 맵 위에 로봇 + waypoint(5개 이상) 표시
%  학번: 201912700
%
%  목표 : 4번에서 만든 캠퍼스 맵 위에 로봇 1대와 경유점(waypoint) 5개
%         이상을 함께 표시한다. (Visualizer2D 에 맵을 입히는 방식)
% =====================================================================

%% [0] 초기화 ---------------------------------------------------------
clear; clc; close all;

%% [1] 맵 만들기 (4번과 동일 + 좌표 다루기 쉽게 1/10 축소) ----------
image = imread('mymap_pknu.png');     % 지도 이미지 읽기
if size(image, 3) == 3                % 컬러면 회색조로
    grayimage = rgb2gray(image);
else
    grayimage = image;
end
bwimage = grayimage < 128;            % 흑백 변환 (검정=장애물)

bwimage_s = imresize(bwimage, 0.1);   % 1/10 축소 → 약 92 x 71 크기로 작아짐
map = binaryOccupancyMap(bwimage_s);  % 점유 격자 지도 생성

%% [2] 시각화 객체에 맵 입히기 -------------------------------------
viz = Visualizer2D;            % 2D 로봇 시각화 객체 생성
viz.mapName = 'map';           % 위에서 만든 'map' 변수를 배경 지도로 사용
viz.showTrajectory = false;    % 궤적선은 끄기 (가만히 표시만 할 거라서)
viz.hasWaypoints = true;       % 경유점 표시 켜기

%% [3] waypoint(경유점) 5개 이상 정의 ------------------------------
%  [x, y] 좌표, 맵 범위(가로 0~92, 세로 0~71) 안에 들어와야 함.
%  ▶ 길(흰색) 위에 오도록 숫자를 조정하세요. (아래는 예시값)
waypoints = [15.19, 19.57;    % 1번 경유점
             43.71, 12.26;    % 2번
             45.07, 34.70;    % 3번
             65.16, 39.78;    % 4번
             58.58, 56.52;    % 5번
             71.11, 49.95];   % 6번 (5개 이상 OK)

%% [4] 로봇 시작 자세 정의 -----------------------------------------
pose = [15.19; 19.57; 0];    % [x; y; theta] — 첫 경유점 위치, 0 rad(오른쪽) 방향

%% [5] 맵 + 로봇 + waypoint 함께 표시 (팝업) ------------------------
viz(pose, waypoints);   % 배경 맵 위에 로봇과 경유점들을 한 번에 표시


%% =====================================================================
%  AutoDrive HW#1 - [보너스] 맵 위에서 waypoint 자율주행 (장애물 회피)
%  학번: 201912700
%
%  과제 범위(5번)를 넘어, 로봇이 건물을 피해 waypoint를 따라 실제로
%  주행하도록 구현. (W11 Pure Pursuit + W13 경로계획 미리 도전)
%
%  파이프라인:
%   [1] 맵 + 안전여유(inflate)
%   [2] 경로계획(PRM) — 건물을 피해 경유점들을 잇는 길 생성
%   [3] 경로추종(Pure Pursuit)
%   [4] 차동구동 로봇 모델 + 시뮬레이션 + GIF 저장
%
%  ※ Navigation Toolbox 기능 사용. 한 번에 안 되면 파라미터 튜닝 필요.
%  ※ 이 섹션 전체를 드래그 선택 후 F9 로 실행하세요. (변수가 이어짐)
% =====================================================================

%% [0] 초기화 ---------------------------------------------------------
clear; clc; close all;

%% [1] 맵 만들기 + 안전여유(inflate) --------------------------------
image = imread('mymap_pknu.png');
if size(image, 3) == 3
    grayimage = rgb2gray(image);
else
    grayimage = image;
end
bwimage   = grayimage < 128;          % 흑백 변환 (검정=장애물)

% 맵 축소 비율. 0.1은 너무 작아 좁은 길이 끊겼음 → 0.25로 키움.
% (값을 키울수록 길 연결은 좋아지지만 계산은 무거워짐)
scale     = 0.35;
bwimage_s = imresize(bwimage, scale, 'nearest'); % 축소 (이진맵은 'nearest'가 적합)
bwimage_s = logical(bwimage_s);           % bwareaopen 입력은 logical 이어야 함

% 잡티 정리: 글자·주차선 등이 작은 '장애물 점'으로 변해 길을 끊는다.
% 작은 덩어리들을 제거해 길 연결성을 살린다. (Image Processing Toolbox)
minBlob   = 30;                            % 이 칸수보다 작은 덩어리는 잡티로 보고 제거
bwimage_s = bwareaopen(bwimage_s, minBlob);   % 길 속 작은 '검은 점' 제거
freeSpace = bwareaopen(~bwimage_s, minBlob);  % 건물 속 작은 '흰 구멍' 정리
bwimage_s = ~freeSpace;

map = binaryOccupancyMap(bwimage_s);  % 표시용 지도

% 계획용으로는 장애물을 로봇 반경만큼 부풀린 '안전 지도'를 따로 만든다.
% (벽에 너무 붙는 경로가 생기지 않게 함)
robotRadius = 0.5;                    % 로봇 반경 [m] (좁은 길 막히면 줄이기)
mapInflated = copy(map);              % 원본은 그대로 두고 복사본을 부풀림
inflate(mapInflated, robotRadius);

%% [2] 경유점 정의 (5번에서 ginput으로 찍은 좌표 재사용) -----------
% 0.1 스케일에서 ginput으로 찍었던 좌표 → 현재 scale로 환산 (×scale/0.1)
waypoints = [15.19, 19.57;
             43.71, 12.26;
             45.07, 34.70;
             65.16, 39.78;
             58.58, 56.52] * (scale / 0.1);

%% [3] 경로계획(PRM): 건물을 피해 경유점들을 잇는 길 만들기 --------
planner = mobileRobotPRM(mapInflated, 3000); % 안전지도 위에 노드 3000개 뿌리기
planner.ConnectionDistance = 40;             % 노드끼리 잇는 최대 거리

route = [];                                  % 전체 경로(점들의 모음)
for i = 1:size(waypoints,1)-1
    seg = findpath(planner, waypoints(i,:), waypoints(i+1,:));  % 두 점 사이 길
    if isempty(seg)
        error(['경로를 못 찾았어요(구간 %d→%d). ' ...
               'robotRadius를 줄이거나 PRM 노드 수를 늘려보세요.'], i, i+1);
    end
    route = [route; seg];                     % 구간 경로를 이어붙임
end

% 경로를 촘촘하게 보간(점 사이 ~2m) → Pure Pursuit이 매끄럽게 따라가고
% 코너에서 오버슈트(후진 못 해서 빙 도는 현상)가 줄어든다.
denseRoute = [];
stepLen = 2;
for i = 1:size(route,1)-1
    p1 = route(i,:);  p2 = route(i+1,:);
    n  = max(1, round(norm(p2 - p1) / stepLen));
    for t = 0:n-1
        denseRoute = [denseRoute; p1 + (p2 - p1) * (t/n)];
    end
end
route = [denseRoute; route(end,:)];

%% [4] 경로추종 컨트롤러(Pure Pursuit) -----------------------------
controller = controllerPurePursuit;
controller.Waypoints = route;                % 따라갈 경로
controller.DesiredLinearVelocity = 3;        % 목표 전진 속도 (낮춰서 경로에 밀착)
controller.MaxAngularVelocity    = 4;        % 최대 회전 속도 (높여서 급커브 대응)
controller.LookaheadDistance     = 5;        % 전방주시거리(작을수록 경로에 딱 붙음)

%% [5] 차동구동 로봇 모델 ------------------------------------------
robot = differentialDriveKinematics( ...
    "TrackWidth", 1, ...                     % 양 바퀴 간격
    "VehicleInputs", "VehicleSpeedHeadingRate");  % 입력: [속도, 회전율]

%% [6] 시뮬레이션 + 시각화 + GIF 저장 ------------------------------
viz = Visualizer2D;
viz.mapName = 'map';          % 배경은 원본 지도(진짜 건물 모양)
viz.hasWaypoints = true;

sampleTime = 0.1;                            % 한 스텝 시간 [s]
% 시작 방향을 첫 경로 방향으로 맞춤 → 출발 직후 빙 도는 현상 방지
theta0 = atan2(route(2,2) - route(1,2), route(2,1) - route(1,1));
pose = [route(1,1); route(1,2); theta0];     % 시작 자세 (경로 첫 점 + 진행방향)
goalRadius = 4;                              % 목적지 도착 판정 반경

gifFile = 'HW1_navigation.gif'; firstFrame = true;

maxSteps  = 6000;                            % 무한루프 방지 안전장치
plotEvery = 5;                               % N스텝마다만 화면 갱신/GIF 저장
                                             %  (클수록 빨리 보임, GIF도 가벼워짐)
for step = 1:maxSteps
    % 목적지(경로 마지막 점)에 충분히 가까우면 종료
    if norm(pose(1:2) - route(end,:)') <= goalRadius
        break;
    end

    [v, w] = controller(pose);               % 컨트롤러가 속도/회전율 결정
    vel  = derivative(robot, pose, [v w]);   % 로봇 모델로 다음 변화량 계산
    pose = pose + vel * sampleTime;          % 자세 갱신(적분) — 물리는 매 스텝 정확히

    % 화면 갱신과 GIF 저장은 N스텝마다만 → 보기 빠르고 GIF 용량 ↓
    if mod(step, plotEvery) == 0
        viz(pose, waypoints);                % 화면 갱신
        drawnow limitrate;                   % 빠른 렌더링

        % --- GIF 한 프레임 저장 ---
        frame = getframe(gcf);
        [A, cmap] = rgb2ind(frame.cdata, 256);
        if firstFrame
            imwrite(A, cmap, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.05);
            firstFrame = false;
        else
            imwrite(A, cmap, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
        end
    end
end
