%% =====================================================================
%  AutoDrive HW#4 - 모바일 로봇 자율주행 (Differential Drive + Lidar)
%  학번: 201912700
%
%  [전체 계획]
%   1단계: 도면(house.png) → Occupancy Grid 맵          ← 지금 여기
%   2단계: Lidar 센서 부착
%   3단계: Object Detector + 장애물 6개 이상
%   4단계: 시작점·목표점 + 자율주행
%          (4a 전역계획 PRM / 4b 지역회피 VFH / 4c 모션제어 Pure Pursuit)
%   5단계: 동작 영상(GIF) 저장
% =====================================================================

%% ====================== 1단계: 맵 만들기 ============================
clc; clear; close all;

% [1] 도면 이미지 읽기 ----------------------------------------------
img = imread('house.png');
if ndims(img) == 3            % 혹시 컬러면 회색조로
    img = rgb2gray(img);
end

% [2] 흑백(이진) 분류 -----------------------------------------------
%  밝은 픽셀(흰 바닥) = 빈 공간,  어두운 픽셀(검은 선=벽·가구) = 장애물
if islogical(img)             % 1비트 이미지면 이미 true(흰)/false(검)
    free = img;
else
    free = img > 128;         % 그레이스케일이면 128 기준
end
occ = ~free;                  % true = 장애물 (벽 + 가구 윤곽선)

% [3] 잡티 제거 + 선 두껍게 + 축소 ---------------------------------
occ = bwareaopen(occ, 5);                  % 작은 점 노이즈 제거
occ = imdilate(occ, strel('disk', 2));     % 얇은 선을 살짝만 두껍게 (축소 대비)
%  ▼ 성능을 위해 축소(셀 수↓ → 라이다/PRM 빨라짐).
%    너무 줄이거나 임계값이 낮으면 통로가 막히므로 0.35 + 0.5 사용.
occ = imresize(double(occ), 0.35) > 0.5;   % 축소 후 재이진화 (0.5: 벽 과팽창 방지)

% [4] Occupancy Grid(점유 격자 지도) 생성 --------------------------
%  축소비 0.35 → resolution 35 로 두면 맵 크기 = 10m x 6.8m 그대로 유지
%  → 기존 좌표(start/goal/객체) 그대로 사용 가능.
resolution = 35;
map = binaryOccupancyMap(occ, resolution);

% [5] 맵 표시 -------------------------------------------------------
figure;
show(map);
title('HW4 - My Map (House Layout)');
xlabel('X [m]'); ylabel('Y [m]');


%% ====================== 2단계: Lidar 센서 ===========================
%  (1단계의 map 변수를 그대로 사용 — 맨 위부터 F5로 실행)

% [1] Lidar 센서 객체 생성 -----------------------------------------
lidar = LidarSensor;
lidar.sensorOffset = [0, 0];               % 로봇 중심에 장착
lidar.scanAngles   = linspace(-pi, pi, 180); % 360도 전방위 스캔(2도 간격, 속도↑)
lidar.maxRange     = 5;                     % 최대 측정 거리 5m
lidar.mapName      = 'map';                 % 어떤 맵을 스캔할지 (변수 이름)

% [2] 로봇을 한 지점에 놓고 한 번 스캔해 보기 -----------------------
pose = [2; 2; 0];          % [x; y; theta]  (거실 근처, 막혔으면 좌표 조정)
ranges = lidar(pose);      % 그 자세에서 라이다가 잰 거리들(360개)

% [3] 시각화 (맵 + 로봇 + 라이다 스캔) -----------------------------
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz, lidar);   % 시각화기에 라이다 연결
viz(pose, ranges);               % 로봇 위치 + 라이다 빔 표시


%% ============= 3단계: Object Detector + 장애물 6개 ==================
%  (앞 단계의 map, lidar, viz, pose, ranges 를 이어서 사용)

% [1] 감지할 객체(장애물) 정의 : N×3 행렬, 각 행 = [x, y, label] ----
%  label = 객체 종류 번호(1,2,3...). 좌표는 빈 공간(방 안)에 두기.
%  ▶ 벽/가구에 겹치면 ginput 으로 빈 곳을 찍어 좌표를 바꾸세요.
objects = [ 1.96, 1.46, 1;     % 1번 장애물
            5.04, 1.48, 2;     % 2번
            5.79, 2.68, 3;     % 3번
            2.07, 3.99, 1;     % 4번
            4.52, 4.28, 2;     % 5번
            8.35, 4.62, 3 ];   % 6번  (요구사항: 6개 이상)

% [2] Object Detector 객체 생성 ------------------------------------
detector = ObjectDetector;
detector.fieldOfView = pi/4;   % 감지 시야각 45도
detector.maxRange    = 4;      % 최대 감지 거리 4m

% [3] 시각화에 연결 + 표시 -----------------------------------------
release(viz);                  % viz가 이미 호출되어 잠김 → 속성 변경 전 해제
attachObjectDetector(viz, detector);
viz(pose, ranges, objects);    % 맵 + 로봇 + 라이다 + 객체(장애물) 함께 표시


%% =============== 4a단계: 전역 경로계획 (Global Planning) ===========
%  객체를 '피해야 할 장애물'로 만들기(B) → 점유 맵에 원형으로 추가

% [1] 6개 객체를 점유 맵에 원형 장애물로 추가 ----------------------
objRadius = 0.2;                              % 객체 물리 반경 [m]
[dx, dy] = meshgrid(-objRadius:1/resolution:objRadius);
mask = (dx.^2 + dy.^2) <= objRadius^2;        % 원형 영역
for i = 1:size(objects,1)
    pts = [objects(i,1)+dx(mask), objects(i,2)+dy(mask)];
    setOccupancy(map, pts, 1);                % 그 칸들을 장애물로
end

% [2] 안전여유: 로봇 반경만큼 부풀린 '계획용 맵' ------------------
robotRadius = 0.12;                           % 로봇 반경 [m] (좁은 통로 막히면 ↓)
mapInflated = copy(map);
inflate(mapInflated, robotRadius);

% [3] 시작점 · 목표점 ----------------------------------------------
start = [1.39, 0.86];   % 시작점 (ginput으로 빈 공간 선택)
goal  = [4.37, 5.22];   % 목표점 (ginput으로 빈 공간 선택)
%  ▶ 막히면 ginput 으로 빈 곳을 골라 좌표를 바꾸세요.

% [4] 전역 경로계획 (PRM) ------------------------------------------
%  PRM은 무작위라 매번 경로가 달라짐. rng로 시드 고정 → 매번 같은 경로.
%  (경로 모양이 마음에 안 들면 rng 숫자를 바꿔가며 돌려서 고른 뒤 고정)
rng(3);
planner = mobileRobotPRM(mapInflated, 1000);
planner.ConnectionDistance = 2;
path = findpath(planner, start, goal);
if isempty(path)
    error('경로를 못 찾았어요 — robotRadius를 줄이거나 노드 수를 늘리세요.');
end

% 경로를 촘촘하게 보간(점 사이 ~0.15m) → Pure Pursuit이 경로에 밀착,
% VFH와 충돌(맴돌이) 방지
denseRoute = [];  stepLen = 0.15;
for i = 1:size(path,1)-1
    p1 = path(i,:);  p2 = path(i+1,:);
    n  = max(1, round(norm(p2-p1)/stepLen));
    for t = 0:n-1
        denseRoute = [denseRoute; p1 + (p2-p1)*(t/n)];
    end
end
path = [denseRoute; path(end,:)];

% [5] 계획된 경로 확인 ---------------------------------------------
figure; show(map); hold on;
plot(path(:,1), path(:,2), 'g-', 'LineWidth', 2);          % 계획 경로
plot(start(1), start(2), 'bo', 'MarkerFaceColor', 'b');    % 시작
plot(goal(1),  goal(2),  'ro', 'MarkerFaceColor', 'r');    % 목표
title('4a - Global Path (PRM)'); xlabel('X [m]'); ylabel('Y [m]');


%% ===== 4b/4c: 지역회피(VFH) + 모션제어(Pure Pursuit) + 주행 =========

% [1] Pure Pursuit : 전역 경로를 따라갈 "목표 방향" 제공 -----------
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.4;
controller.MaxAngularVelocity    = 2;
controller.LookaheadDistance     = 0.3;

% [2] VFH : 라이다로 주변을 보고 "안전한 조향 방향" 계산(지역회피) -
vfh = controllerVFH;
vfh.DistanceLimits   = [0.05, 1.5];   % 가까운(1.5m) 장애물만 고려 (먼 벽까지 보면 막힘)
vfh.RobotRadius      = 0.1;           % 로봇 반경
vfh.SafetyDistance   = 0.05;          % 안전 여유
vfh.MinTurningRadius = 0.1;           % 최소 회전 반경

% [3] 차동구동 로봇 모델 -------------------------------------------
robot = differentialDriveKinematics("TrackWidth", 0.3, ...
        "VehicleInputs", "VehicleSpeedHeadingRate");

% [4] 시각화 새로 구성 + 시뮬레이션 설정 ---------------------------
%  앞 단계의 viz가 꼬일 수 있어, 주행용 viz를 여기서 새로 만든다.
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz, lidar);
attachObjectDetector(viz, detector);

sampleTime = 0.1;
theta0 = atan2(path(2,2)-path(1,2), path(2,1)-path(1,1)); % 시작 방향
pose   = [start(1); start(2); theta0];
goalRadius = 0.2;
maxW = 2;  Kp = 4;                 % 각속도 제한 / 조향 비례이득
scanAngles = lidar.scanAngles(:);  % 라이다 각도(열벡터)

gifFile = 'HW4_navigation.gif'; firstFrame = true;
plotEvery = 5;                     % N스텝마다 화면/GIF (빠르게 보기)
maxSteps  = 4000;

% [5] 주행 루프 ----------------------------------------------------
for step = 1:maxSteps
    % 목표 도착 판정
    if norm(pose(1:2)' - goal) <= goalRadius
        disp('목표 지점에 도착했습니다!');
        break;
    end

    ranges = lidar(pose);                       % 라이다 스캔

    % (전역) 경로 추종 → 목표 방향
    [~, ~, lookPt] = controller(pose);
    d = atan2(lookPt(2)-pose(2), lookPt(1)-pose(1)) - pose(3);
    targetDir = atan2(sin(d), cos(d));          % -pi~pi 로 정규화

    % (지역) VFH 회피 → 안전 조향 방향
    steerDir = vfh(ranges, scanAngles, targetDir);

    if isnan(steerDir)            % 갈 곳 없으면 제자리 회전
        v = 0;  w = maxW;
    else                          % 안전 방향으로 조향
        v = 0.4;
        w = max(min(Kp*steerDir, maxW), -maxW);
    end

    % 로봇 운동 적분
    vel  = derivative(robot, pose, [v w]);
    pose = pose + vel * sampleTime;

    % 화면 갱신 + GIF (N스텝마다)
    if mod(step, plotEvery) == 0
        viz(pose, ranges, objects);
        drawnow limitrate;
        frame = getframe(gcf);
        [A, cmap] = rgb2ind(frame.cdata, 256);
        if firstFrame
            imwrite(A, cmap, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.1);
            firstFrame = false;
        else
            imwrite(A, cmap, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end
    end
end
