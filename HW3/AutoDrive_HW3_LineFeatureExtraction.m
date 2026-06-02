%% =====================================================================
%  AutoDrive HW#3 - 라이다 Range Data 기반 Line Feature Extraction
%  학번: 201912700
%
%  목표 : 라이다 측정값(조사각 θ, 거리 ρ)으로부터 직선(벽) feature를
%         극좌표 normal form (r, α)로 추출한다.
%         - r : 원점(로봇)에서 직선까지의 수직 거리
%         - α : 그 수직선이 x축과 이루는 각도
%         강의자료 정답: α ≈ 37.36°, R ≈ 0.4
%
%  두 가지 방법을 비교한다:
%   [방법 A] polyfit        : y방향 오차 최소화 (일반 최소제곱)
%   [방법 B] 강의자료 공식  : 수직거리 오차 최소화 (Total Least Squares) ← 정석
% =====================================================================

clear; clc; close all;

%% [1] 라이다 측정 데이터 -------------------------------------------
theta_deg = (0:5:80)';          % 센서 조사각 [deg], 0~80도 5도 간격 = 17개
theta     = deg2rad(theta_deg); % 라디안으로 변환 (삼각함수는 라디안 사용)
rho = [0.5197 0.4404 0.485 0.4222 0.4132 0.4371 0.3912 0.3949 0.3919 ...
       0.4276 0.4075 0.3956 0.4053 0.4752 0.5032 0.5273 0.4879]';  % 거리 [m]

% 극좌표(거리·각도) → 직교좌표(x, y) :  x = ρcosθ,  y = ρsinθ
x = rho .* cos(theta);
y = rho .* sin(theta);

n = numel(rho);                 % 측정점 개수

%% [2] 방법 A : polyfit (y방향 최소제곱) ----------------------------
%  직선을 y = a*x + b 로 보고, y방향(수직) 오차를 최소화한다.
p = polyfit(x, y, 1);           % p(1)=기울기 a, p(2)=절편 b
a = p(1);  b = p(2);

alpha_A = rad2deg(atan(-1/a));      % 직선의 normal 각도 [deg]
r_A     = abs(b) / sqrt(a^2 + 1);   % 원점~직선 수직거리 (정식 변환식)
%   ※ 기존 코드의 b/sqrt(a^2+1+b^2) 는 +b^2 가 잘못 들어간 식이었음 → 수정

fprintf('[방법 A: polyfit]        alpha = %8.4f deg,  r = %.4f m\n', alpha_A, r_A);

%% [3] 방법 B : 강의자료 공식 (Total Least Squares) -----------------
%  S = Σ (ρ_i cos(θ_i - α) - r)^2  (직선까지의 수직거리 제곱합) 을 최소화한
%  닫힌 해(closed-form). 라인 피팅의 정석.
num = sum(rho.^2 .* sin(2*theta)) ...
      - (2/n) * sum(rho.*cos(theta)) * sum(rho.*sin(theta));
den = sum(rho.^2 .* cos(2*theta)) ...
      - (1/n) * sum(sum( (rho*rho') .* cos(theta + theta') ));

alpha_B = 0.5 * atan2(-num, -den);             % [rad] (부호 주의: -num, -den)
r_B     = sum(rho .* cos(theta - alpha_B)) / n;% [m]

fprintf('[방법 B: 강의공식(TLS)]  alpha = %8.4f deg,  r = %.4f m\n', ...
        rad2deg(alpha_B), r_B);
fprintf('(강의자료 정답: alpha = 37.36 deg, R = 0.4)\n');

%% [4] 결과 시각화 --------------------------------------------------
figure; hold on; grid on; axis equal;

% 라이다 측정점
plot(x, y, 'ko', 'MarkerFaceColor', 'k');

% 방법 A 직선 (y = a x + b)
xx = linspace(min(x)-0.05, max(x)+0.05, 100);
plot(xx, a*xx + b, 'b--', 'LineWidth', 1.5);

% 방법 B 직선 : normal form 을 점으로 그림
%   직선 위 최근접점 = r*(cosα, sinα), 직선 방향 = (-sinα, cosα)
t = linspace(-0.4, 0.4, 100);
xB = r_B*cos(alpha_B) - t*sin(alpha_B);
yB = r_B*sin(alpha_B) + t*cos(alpha_B);
plot(xB, yB, 'r-', 'LineWidth', 1.5);

legend('라이다 측정점', '방법 A: polyfit', '방법 B: 강의공식(TLS)', ...
       'Location', 'best');
title('Lidar Line Feature Extraction');
xlabel('X [m]'); ylabel('Y [m]');
