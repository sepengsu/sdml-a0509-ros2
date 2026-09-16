# Unity에 DH AG145 URDF Import하기

## 확인된 경로

| 구분 | 경로 |
| --- | --- |
| Unity 프로젝트 | `D:\Jaewon\60th\My project` |
| Unity Assets 폴더 | `D:\Jaewon\60th\My project\Assets` |
| 복사할 원본 폴더 | `D:\Jaewon\60th\sdml-doosan-robot\python\dh_ag145` |
| 권장 Unity 배치 경로 | `D:\Jaewon\60th\My project\Assets\Urdf\dh_ag145` |
| Unity에서 선택할 URDF | `Assets\Urdf\dh_ag145\dh_ag145.urdf` |

Unity 프로젝트의 `Packages\manifest.json`에는 공식 URDF Importer `v0.5.2`가 이미
등록되어 있다.

```json
"com.unity.robotics.urdf-importer": "https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer#v0.5.2"
```

## 1. URDF와 메시 복사

Unity Editor를 닫은 상태에서 PowerShell로 아래 명령을 실행한다.

```powershell
$source = "D:\Jaewon\60th\sdml-doosan-robot\python\dh_ag145"
$destination = "D:\Jaewon\60th\My project\Assets\Urdf\dh_ag145"

New-Item -ItemType Directory -Force -Path $destination
Copy-Item -Path "$source\*" -Destination $destination -Recurse -Force
```

복사 후 다음 구조가 있어야 한다.

```text
Assets/
  Urdf/
    dh_ag145/
      dh_ag145.urdf
      meshes/
        base_link.stl
        body.stl
        left_finger.stl
        ...
```

현재 URDF는 `meshes/base_link.stl`처럼 상대 경로를 사용한다. 따라서
`dh_ag145.urdf`와 `meshes` 폴더의 상대 위치를 유지해야 한다.

## 2. Unity Editor에서 Import

1. Unity Hub에서 `D:\Jaewon\60th\My project`를 연다.
2. 패키지 설치가 끝날 때까지 기다린다.
3. Project 창에서 `Assets/Urdf/dh_ag145/dh_ag145.urdf`를 찾는다.
4. `dh_ag145.urdf`를 마우스 오른쪽 버튼으로 클릭한다.
5. `Import Robot from Selected URDF file`을 선택한다.
6. Import 창에서 메시 방향을 확인한다. STL 방향이 어긋나면 `Y Axis` 또는
   `Z Axis` 설정을 바꾸어 미리보기를 확인한다.
7. `Import URDF`를 누른다.
8. 생성된 프리팹 또는 로봇 GameObject를 Scene으로 끌어 놓는다.

공식 문서도 URDF와 관련 메시 파일을 Unity `Assets` 아래에 복사하고, URDF를
오른쪽 클릭하여 import하도록 안내한다.

## 3. Import 후 확인

Hierarchy에서 로봇 링크 구조가 아래처럼 생성되었는지 확인한다.

```text
world
  base_link
    body_link
      left_outer_knuckle
        left_finger
      left_inner_knuckle
      right_outer_knuckle
        right_finger
      right_inner_knuckle
```

확인할 항목:

- STL 메시 8개가 빠짐없이 보이는가?
- 스케일이 지나치게 크거나 작지 않은가?
- `left_outer_knuckle_joint`를 움직였을 때 손가락 방향이 맞는가?
- Console 창에 메시 파일을 찾지 못했다는 오류가 없는가?

## 4. 현재 URDF의 제한 사항

`python\urdf_test.py`로 확인한 현재 상태:

```text
[PASS] STL 메시 경로 8개 정상
[WARN] collision 형상 없음
[WARN] inertial 데이터 없음
[WARN] mimic 조인트 5개 존재
[INFO] ros2_control은 Unity 제어 코드로 대체 필요
```

따라서 현재 파일은 Unity에서 외형을 import하고 관절 구조를 확인하는 용도로
사용할 수 있다. 실제 물리 기반 그리퍼 시뮬레이션에는 추가 작업이 필요하다.

- 충돌이 필요하면 각 링크에 `<collision>` 요소를 추가한다.
- 물리 시뮬레이션이 필요하면 각 링크에 `<inertial>` 요소를 추가한다.
- mimic 조인트가 Unity에서 자동으로 연동되지 않으면 C# 스크립트에서
  `left_outer_knuckle_joint` 값을 나머지 손가락 관절에 multiplier와 함께 적용한다.
- ROS 명령과 연결하려면 Unity `ArticulationBody` drive 또는 ROS-Unity 제어
  스크립트를 작성한다.

## 5. 다시 검증하는 명령

URDF를 수정한 뒤에는 아래 명령으로 메시 경로와 FK를 다시 확인한다.

```powershell
cd "D:\Jaewon\60th\sdml-doosan-robot"
.\.venv\Scripts\python.exe python\urdf_test.py
```

## 참고 자료

- [Unity URDF Importer 공식 저장소](https://github.com/Unity-Technologies/URDF-Importer)
- [Unity URDF Importer v0.5.2 릴리스](https://github.com/Unity-Technologies/URDF-Importer/releases/tag/v0.5.2)
