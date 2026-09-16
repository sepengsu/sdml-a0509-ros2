# Unity ↔ 로봇(ROS 2) rosbridge 연동 가이드

로봇 쪽 server_bridge는 **서버(ROS 1)에서는 표준 메시지만 사용**하도록 되어 있습니다.  
Unity에서 `/do_pick_and_place`, `/jog_multi_axis` 를 보낼 때 **커스텀 타입 대신 `std_msgs/String` + JSON** 을 사용해야 합니다.

---

## 1. 왜 std_msgs/String 인가?

- rosbridge 서버는 **ROS 1 (Noetic)** 이며, `pick_and_place_msgs`, `dsr_msgs2` 패키지가 설치되어 있지 않습니다.
- 따라서 Unity에서 **advertise / publish** 시 `pick_and_place_msgs/PickandPlace`, `dsr_msgs2/JogMultiAxis` 타입을 쓰면  
  `Unable to load the manifest for package ...` 에러가 납니다.
- **해결**: 두 토픽 모두 **타입을 `std_msgs/String`** 으로 하고, 메시지 내용을 **JSON 문자열**로 넣습니다.  
  로봇 쪽 server_bridge가 이 JSON을 파싱해 ROS 2의 PickandPlace / JogMultiAxis 로 변환합니다.

---

## 2. Unity에서 수정할 내용 요약

| 항목 | 기존 (에러 유발) | 수정 후 |
|------|------------------|---------|
| advertise 타입 | `pick_and_place_msgs/PickandPlace`, `dsr_msgs2/JogMultiAxis` | **`std_msgs/String`** |
| publish 시 msg | 커스텀 메시지 객체 | **`{ "data": "<JSON 문자열>" }`** |

- **advertise**: 토픽 이름은 그대로 두고, **type 만 `std_msgs/String`** 으로 보냅니다.
- **publish**: `msg` 필드에 **`{ "data": "..." }`** 형태로, `data` 에 아래 JSON 형식의 **문자열**을 넣습니다.

---

## 3. rosbridge 메시지 예시

### 3.1 advertise (연결 후 한 번만)

**/do_pick_and_place**

```json
{
  "op": "advertise",
  "topic": "/do_pick_and_place",
  "type": "std_msgs/String"
}
```

**/jog_multi_axis**

```json
{
  "op": "advertise",
  "topic": "/jog_multi_axis",
  "type": "std_msgs/String"
}
```

### 3.2 publish — 공통 형식

모든 publish 에서 **msg** 는 반드시 **std_msgs/String** 형식이어야 합니다.

```json
{
  "op": "publish",
  "topic": "<토픽 이름>",
  "msg": {
    "data": "<아래 형식의 JSON 문자열>"
  }
}
```

`data` 는 **문자열**이어야 하며, 그 내용이 아래의 PickandPlace / JogMultiAxis JSON입니다.

---

## 4. /do_pick_and_place — "go" 트리거만 전송

토픽에는 **명령 문자열만** 보냅니다. start_pose / target_pose 는 **로봇 쪽 JSON 파일**(`~/.ros/pick_and_place/scenario_positions.json`)에서만 읽습니다.

**msg.data 에 넣을 내용**: 문자열 **`"go"`** 만 보내면 됩니다.

```json
{
  "op": "publish",
  "topic": "/do_pick_and_place",
  "msg": { "data": "go" }
}
```

- Unity에서 `data` 필드에 **"go"** 문자열만 넣어 publish 하면, server_bridge가 ROS 2의 `PickandPlace(go="go")` 로 변환하고, pick_and_place 노드가 JSON 파일의 두 pose 를 읽어 scenario 를 실행합니다.

---

## 5. /jog_multi_axis — JSON 형식 (data 문자열 내용)

`msg.data` 에 들어갈 **JSON 문자열** 한 덩어리입니다.

```json
{
  "jog_axis": [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
  "move_reference": 0,
  "speed": 20.0
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| jog_axis | float[6] | 작업 공간 단위 벡터 [Tx, Ty, Tz, Rx, Ry, Rz], -1.0 ~ 1.0 |
| move_reference | int | 0: BASE, 1: TOOL, 2: WORLD |
| speed | float | 조그 속도 [%] |

**정지**: `jog_axis` 를 `[0,0,0,0,0,0]`, `speed` 를 `0` 으로 보내면 됩니다.

**Unity에서 보낼 때**: 위 구조를 JSON 문자열로 직렬화한 뒤, 그 문자열을 `msg.data` 에 넣어 publish 합니다.

---

## 6. Unity C# 예시 (개념)

```csharp
// advertise (연결 직후 1회)
void AdvertiseTopics() {
    SendToRosbridge(new {
        op = "advertise",
        topic = "/do_pick_and_place",
        type = "std_msgs/String"
    });
    SendToRosbridge(new {
        op = "advertise",
        topic = "/jog_multi_axis",
        type = "std_msgs/String"
    });
}

// Pick and Place 트리거 (pose 는 로봇 쪽 JSON 파일에서 로드)
void PublishPickAndPlaceGo() {
    SendToRosbridge(new {
        op = "publish",
        topic = "/do_pick_and_place",
        msg = new { data = "go" }
    });
}

// Jog 발행
void PublishJog(float[] jogAxis, int moveReference, float speed) {
    var payload = new {
        jog_axis = jogAxis,
        move_reference = moveReference,
        speed = speed
    };
    string jsonString = JsonUtility.ToJson(payload);
    SendToRosbridge(new {
        op = "publish",
        topic = "/jog_multi_axis",
        msg = new { data = jsonString }
    });
}
```

- `SendToRosbridge` 는 위 객체를 JSON으로 직렬화해 WebSocket으로 전송하는 함수로 가정합니다.
- 중첩 객체가 있으면 Unity `JsonUtility` 는 필드명을 정확히 맞추거나, Newtonsoft.Json 등 사용을 권장합니다.

---

## 7. 체크리스트

- [ ] **advertise** 시 `/do_pick_and_place`, `/jog_multi_axis` 타입을 **`std_msgs/String`** 으로 변경했는가?
- [ ] **publish** 시 `msg` 를 **`{ "data": "<문자열>" }`** 형태로 보내는가?
- [ ] `/do_pick_and_place` 에는 **`data: "go"`** 만 보내는가? (pose 는 로봇 쪽 scenario_positions.json 사용)
- [ ] JogMultiAxis JSON에 `jog_axis`(배열 6개), `move_reference`, `speed` 가 포함되는가?

이렇게 수정하면 서버(ROS 1)에서는 커스텀 패키지 없이 동작하고, 로봇 쪽 server_bridge가 JSON을 ROS 2 메시지로 변환합니다.
