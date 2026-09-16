## do_scenario.json 만드는 법

`do_scenario.json`은 **pick_and_place** 패키지의 `do_position_based_scenerio()`가 읽는 시나리오 파일입니다.  
형식은 **JSON 배열(list)** 이고, 각 원소는 하나의 스텝(step)을 나타내는 **객체(dict)** 입니다.

```json
[
  { "posj": [ ... ], "time": 2.0 },
  { "posx": [ ... ], "gripper": 50.0 },
  { "movec": [ [ ... ], [ ... ] ], "movec_time": 4.0 }
]
```

---

## 1. 기본 규칙

- **파일 위치**: `json/do_scenario.json`
- **JSON 최상위 타입**: 리스트(`[]`)
- 리스트 안의 각 원소는 **딕셔너리(객체)** 여야 합니다.
- 한 스텝에는 **아래 타입 중 하나만** 넣는 것이 좋습니다.
  - `posx`: 데카르트 좌표 이동
  - `posj`: 관절 좌표 이동
  - `movec`: 원호 이동 (MoveC)
  - `gripper`: 그리퍼 열림 %

잘못된 예 (타입이 없는 항목, 리스트가 아닌 최상위 구조 등)는 로그에 `"Invalid position: ..."` 으로 에러가 찍힙니다.

---

## 2. posx 스텝 (task space 직선 이동)

```json
{
  "posx": [x, y, z, rx, ry, rz],
  "time": 2.0
}
```

- `posx`: 길이 6 리스트
  - `[x, y, z, rx, ry, rz]`
  - 단위: `x,y,z`는 **mm**, `rx,ry,rz`는 **deg**
- 시간 옵션
  - `time`: 이 스텝 전체에 공통으로 쓰는 시간(초)
  - 또는 `posx_time`: posx 전용 시간(초)
- 내부 동작
  - `position_based_scenerio.py` → `moveline(pose, time_sec=...)` 호출

예:

```json
{
  "posx": [400.0, 500.0, 800.0, 0.0, 180.0, 0.0],
  "time": 3.0
}
```

---

## 3. posj 스텝 (joint space 이동)

```json
{
  "posj": [j1, j2, j3, j4, j5, j6],
  "posj_time": 2.0
}
```

- `posj`: 길이 6 리스트
  - `[j1, j2, j3, j4, j5, j6]`
  - 단위: **deg**
- 시간 옵션
  - `time`: 공통 시간(초)
  - 또는 `posj_time`: posj 전용 시간(초)
- 내부 동작
  - `position_based_scenerio.py` → `movejoint(joint, time_sec=...)` 호출

예:

```json
{
  "posj": [0, 0, 90, 0, 90, 0],
  "posj_time": 2.5
}
```

---

## 4. movec 스텝 (원호 이동: MoveC)

```json
{
  "movec": [
    [via_x, via_y, via_z, via_rx, via_ry, via_rz],
    [end_x, end_y, end_z, end_rx, end_ry, end_rz]
  ],
  "movec_time": 4.0
}
```

- `movec`: 길이 2인 리스트
  - 첫 번째 원소: **경유점(via) posx 6D**
  - 두 번째 원소: **종료점(end) posx 6D**
  - 각 요소는 `[x, y, z, rx, ry, rz]` (mm, deg)
- 시간 옵션
  - `time`: 공통 시간(초)
  - 또는 `movec_time`: movec 전용 시간(초), 기본 3.0
- 내부 동작
  - `position_based_scenerio.py` → `movecircle(pos1, pos2, time_sec=...)` 호출

예:

```json
{
  "movec": [
    [400.0, 500.0, 800.0, 0.0, 180.0, 0.0],
    [400.0, 600.0, 800.0, 0.0, 180.0, 0.0]
  ],
  "movec_time": 5.0
}
```

---

## 5. gripper 스텝 (그리퍼 열림 %)

```json
{
  "gripper": 50.0
}
```

- `gripper`: 0.0 ~ 100.0 (%)
  - 0: 완전히 닫힘, 100: 완전히 열림 (실제 매핑은 드라이버 설정에 따름)
- 내부 동작
  - `position_based_scenerio.py` → `node.set_gripper(%)` 호출 후 약간 대기

---

## 6. 시간 옵션 정리

각 스텝 객체에서 시간을 줄 수 있는 키:

- 공통:
  - `time`: 이 스텝의 동작에 공통 적용
- 타입별:
  - `posx_time`: posx 전용
  - `posj_time`: posj 전용
  - `movec_time`: movec 전용

우선순위:

1. `time` 키가 있으면 **공통 시간**으로 사용
2. 없으면 해당 타입 전용 키 사용  
   - posx → `posx_time` (기본 2.0)  
   - posj → `posj_time` (기본 2.0)  
   - movec → `movec_time` (기본 3.0)

그리퍼 스텝은 시간 옵션을 쓰지 않습니다.

---

## 7. 전체 예시

```json
[
  {
    "posj": [0, 0, 90, 0, 90, 0],
    "posj_time": 2.0
  },
  {
    "posx": [400.0, 500.0, 800.0, 0.0, 180.0, 0.0],
    "time": 3.0
  },
  {
    "movec": [
      [400.0, 500.0, 800.0, 0.0, 180.0, 0.0],
      [400.0, 600.0, 800.0, 0.0, 180.0, 0.0]
    ],
    "movec_time": 5.0
  },
  {
    "gripper": 50.0
  }
]
```

이 파일을 `json/do_scenario.json` 으로 저장한 뒤, GUI에서 **Do Scenario (do_pick_and_place)** 버튼을 누르면 위 순서대로:

1. posj 위치로 이동  
2. posx 직선 이동  
3. MoveC 원호 이동  
4. 그리퍼 50%  

이 실행됩니다.

