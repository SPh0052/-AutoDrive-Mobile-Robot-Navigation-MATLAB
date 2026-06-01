# AutoDrive HW#1 — Robot Visualizer (Mobile Robot Navigation, MATLAB)

> MATLAB **Mobile Robotics Simulation Toolbox**로 구현한 2D 모바일 로봇의
> **경로 주행(사각형·원) · 실제 지도 기반 점유 격자 맵 생성 · waypoint 자율주행
> (전역 경로계획 + Pure Pursuit)** 프로젝트.

자율주행 시스템(Autonomous Driving System) 강의의 HW#1 "Robot Visualizer I (Waypoint, Map)"를
구현하고, 과제 범위를 넘어 **맵 위 자율주행**까지 추가로 도전했다.

**📅 수행 기간:** 2024.04.04 ~ 2024.04.18
**📚 과목:** 자율주행 시스템 (Autonomous Driving System)
**💻 MATLAB 버전:** R2022b

---

## 🛠 사용 기술

- **언어/환경**: MATLAB
- **툴박스**: Mobile Robotics Simulation Toolbox, Image Processing Toolbox, Navigation Toolbox
- **핵심 함수/개념**:
  `Visualizer2D` · `binaryOccupancyMap` · `inflate` · `mobileRobotPRM` ·
  `controllerPurePursuit` · `differentialDriveKinematics` ·
  점유 격자(Occupancy Grid) · 전역 경로계획(PRM) · 경로추종(Pure Pursuit)

## 📂 파일

| 파일 | 설명 |
|------|------|
| `AutoDrive_HW1_RobotVisualizer.m` | 전체 코드 (작업별 `%%` 섹션으로 구성) |
| `TROUBLESHOOTING.md` | 개발·트러블슈팅 기록 / 의문점 Q&A / 한계 분석 |
| `mymap_pknu.png` | 맵 원본 (부경대 캠퍼스, 네이버 지도 기반) |

---

## 1. 사각형(Rectangular) 경로 주행

로봇이 사각형 경로를 따라 이동하며 **항상 진행 방향을 바라본다**(`θ = atan2(dy, dx)`).

![사각형 경로 주행](HW1_1.gif)

## 2. 원(Circular) 경로 주행

원 둘레를 따라 이동하며 **접선 방향을 바라본다**(`θ = φ + π/2`).

![원 경로 주행](HW1_2.gif)

## 3. 실제 지도 → 점유 격자 맵 (Occupancy Grid)

네이버 지도(부경대 캠퍼스) 캡쳐를 `imread → rgb2gray → 임계값 → binaryOccupancyMap`으로
점유 격자 지도로 변환해 표시한다. (흰색 = 통행 가능, 검은색 = 장애물)

![지도 표시](HW1_3-4.png)

## 4. 맵 위 로봇 + waypoint 표시

`Visualizer2D`에 위 맵을 입히고(`viz.mapName`), 로봇과 경유점(5개 이상)을 함께 표시한다.
경유점 좌표는 `ginput`으로 맵에서 직접 클릭해 지정했다.

## 5. (보너스) 맵 위 자율주행 — 장애물 회피 시도

과제 범위를 넘어, 로봇이 경유점을 따라 **실제로 주행**하도록 구현했다.

- **안전여유**: `inflate`로 장애물을 로봇 반경만큼 부풀린 안전 지도 생성
- **전역 경로계획**: `mobileRobotPRM`로 건물을 피해 경유점들을 잇는 경로 생성
- **경로추종**: `controllerPurePursuit` + `differentialDriveKinematics`로 주행 시뮬레이션

![맵 위 자율주행](HW1_navigation.gif)

> ⚠️ **한계**: 계획된 경로는 장애물을 피하지만, Pure Pursuit은 **전진 전용이며
> 주행 중 충돌 검사가 없어** 급커브에서 경로를 벗어나 일부 벽을 침범한다.
> 원인 분석과 개선 방향(HW4에서 Lidar 기반 지역 회피로 보완)은
> **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** 에 정리했다.

---

## ▶ 실행 방법

1. MATLAB에서 `AutoDrive_HW1_RobotVisualizer.m`을 연다.
2. 작업은 `%%` 섹션으로 나뉘어 있다.
   - 한 섹션 실행: 해당 섹션에 커서 두고 **Ctrl + Enter**
   - 여러 섹션이 변수를 공유하는 작업(5번·보너스): 해당 블록 **전체 선택 후 F9**
3. 실행하면 시각화 창이 뜨고, 각 작업별 GIF가 작업 폴더에 저장된다.

> 필요 툴박스: Mobile Robotics Simulation Toolbox, Image Processing Toolbox, Navigation Toolbox

---

## 🔎 한계 & 다음 단계

이 프로젝트는 **자율주행 파이프라인(계획 → 추종 → 제어)을 이해·구현한 데모**다.
주행 중 실시간 충돌 회피(Lidar 기반 지역 계획)는 포함되지 않아, 이는 후속 과제(HW#4)에서
**Lidar + Object Detector + local path planning**으로 보완할 예정이다.
자세한 분석은 [TROUBLESHOOTING.md](TROUBLESHOOTING.md) 참고.
