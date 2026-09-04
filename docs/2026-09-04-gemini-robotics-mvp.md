# Gemini Robotics 카메라 MVP — 2026-09-04

## 오늘 완료한 범위

- Astra RGB, Astra Depth, 손목 카메라 2개를 하나의 2×2 OpenCV 창으로 표시한다.
- Gemini Robotics-ER 2 Preview로 **Astra RGB만** 객체 포인트와 라벨을 탐지한다. 손목 카메라와 depth 패널은 원본 영상 그대로다.
- `camera_preview_gemini.py`는 Gemini 호출을 백그라운드 스레드에서 수행한다. 따라서 API 응답(실측 약 5–6초)을 기다리는 동안 창의 카메라 영상은 멈추지 않는다.
- `gemini_point_snapshot.py`는 Astra RGB와 손목 카메라 2개에서 한 장씩 캡처해 라벨이 그려진 PNG를 `/tmp/gemini_points_*.png`에 저장한다.

## 추가된 실행 파일

- `camera_preview_gemini.py` — 실시간 2×2 미리보기와 Astra RGB Gemini 오버레이
- `gemini_point_snapshot.py` — 3개 RGB 피드의 일회성 Gemini 라벨 스냅샷

## 실행

먼저 Astra watchdog을 별도 터미널에서 실행해 `/tmp/vsp_astra_rgb.png`와 depth 프레임을 publish한다. 이 장비에서는 watchdog과 preview가 동시에 Astra 장치를 직접 열면 안 된다.

```bash
~/lerobot_song_venv/bin/python ~/so101-bimanual-teleop/gemini_point_snapshot.py --self-test
~/lerobot_song_venv/bin/python ~/so101-bimanual-teleop/camera_preview_gemini.py
```

`GEMINI_API_KEY`와 `google-genai` 패키지가 필요하다. 창이 뜬 뒤 첫 Gemini 응답까지 약 5–6초가 걸린다. `q`로 종료한다.

## Gemini API 계약

- 모델: `gemini-robotics-er-2-preview`
- 입력: PNG 이미지와 객체 포인팅 JSON 요청 프롬프트
- 응답: `[{"point": [y, x], "label": "..."}]`
- 좌표: `y`, `x`는 이미지 높이·너비에 대해 0–1000 정규화 값이며, 표시할 때 각각 `y * height / 1000`, `x * width / 1000`로 변환한다.

## 검증 결과

- 스냅샷 경로에서 Astra RGB 객체 탐지가 확인되었다.
- 라이브 worker는 첫 응답 뒤 탐지 포인트 목록을 갱신하며, 화면 렌더링은 네트워크 호출과 분리되어 있다.
- 실제 GUI는 장비가 연결된 데스크톱에서 확인해야 한다.

## 다음 작업 — 아직 구현하지 않음

요청한 **Gemini 라벨이 표시된 `camera_preview_gemini.py` 창에서 클릭해 양팔 IK 파지를 시작하는 연결**은 아직 합치지 않았다. 현재 클릭-파지 기능은 LeRobot 작업 트리의 `custom_scripts/vision_pick_place/task_red_cube_to_bin_new_gripper/click_grasp_bimanual.py`에 별도 창으로 있다. 다음 세션에서는 Gemini 오버레이 창의 클릭 이벤트를 이 안전 파이프라인에 연결한다.

실물 팔을 움직이는 코드는 오른팔 보정값이 아직 placeholder라 오른쪽 클릭을 기본 잠금하고, `e` 키는 양팔 토크를 즉시 해제하며 이후 홈 복귀 명령도 보내지 않도록 구성되어 있다. 처음 통합 시험은 반드시 작업자 감독, 비상정지 접근 가능, 작업공간 비움 상태에서 한다.
