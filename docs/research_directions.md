# 랩스케일 코봇 연구 방향성 문헌 조사 보고서

## 장비 구성: Doosan A0509 (F/T Sensorless) + DH-Robotics AG-145

---

## 1. 서론: 현재 장비의 특성과 연구 가능성

### 1.1 장비 개요

**Doosan A0509**는 6축 협동로봇으로 5kg 페이로드, 900mm 리치, ±0.03mm 반복정밀도를 제공합니다. F/T Sensorless 모델은 엔드이펙터 끝단의 Force/Torque 센서가 없는 대신, 각 관절의 토크 센서를 통해 충돌 감지와 기본적인 힘 제어가 가능합니다. ROS 호환이 되므로 연구용으로 풍부한 소프트웨어 생태계를 활용할 수 있습니다.

**DH-Robotics AG-145**는 적응형 병렬 그리퍼로, 최대 145mm 스트로크와 적응형 파지 기능을 제공합니다. 다양한 형상과 크기의 물체를 잡을 수 있으며, 전기 구동 방식으로 그립력 조절이 가능합니다.

### 1.2 F/T Sensorless의 연구적 의미

F/T 센서가 없다는 것은 단순한 제약이 아니라 오히려 **"센서 의존도를 낮춘 지능형 조작(Sensorless Intelligent Manipulation)"**이라는 중요한 연구 주제와 직접 연결됩니다. 최근 문헌에서는 외부 센서 없이도 비전 기반 힘 추정, 모델 기반 외란 관측기(Disturbance Observer), 학습 기반 접촉 인식 등을 통해 정교한 조작이 가능함을 보이고 있습니다. 이 장비 구성은 이러한 연구를 수행하기에 매우 적합한 테스트베드입니다.

---

## 2. 연구 방향 1: 비전 기반 Sensorless 힘 추정 및 적응형 파지

### 2.1 배경 및 동기

전통적으로 섬세한 물체의 안전한 파지에는 F/T 센서가 필수적이었습니다. 그러나 외부 센서는 비용, 배선 복잡도, 환경 제약 등의 한계가 있습니다. 최근 연구에서는 컴퓨터 비전만으로 접촉력을 추정하는 방법들이 활발히 연구되고 있습니다.

### 2.2 핵심 문헌

**비전 기반 힘 추정**: Xu et al. (2025, *Biomimetics*)은 SuperPoint-SIFT 특징 융합과 유한요소해석(FEA)을 결합하여 변형 가능한 물체의 파지력을 센서 없이 추정하는 방법을 제안했습니다. 3D 변위장을 시각적으로 재구성하고 FEA로 접촉력을 계산하는 방식으로, 정밀 조립과 수술 로봇 등에서의 가치를 입증했습니다.

**비전 기반 가변 강성 그리퍼**: Fu et al. (2024, *Robotica*)는 ArUco 마커를 활용한 비전 기반 힘 감지 방법으로 전통적 힘 센서를 대체하고, 폐루프 제어를 통해 다양한 경도와 취약성의 물체를 안전하게 파지하는 전략을 제시했습니다.

**학습 기반 적응형 파지**: Kadalagere Sampath et al. (2025, *Applied Sciences*)는 YOLOv8 객체 감지, Gaussian Process, Transformer 모델을 결합하여 95.6%의 파지 성공률을 달성하는 프레임워크를 보고했습니다.

### 2.3 제안 연구 주제

A0509의 관절 토크 센서 데이터와 외부 카메라(RGB-D)를 결합하여, AG-145 그리퍼가 잡고 있는 물체의 파지 상태를 실시간으로 추정하는 멀티모달 파지 안정성 예측 시스템을 개발할 수 있습니다. 특히 변형 가능한 물체(식품, 생체 시료 등)에 대해 비전 피드백으로 그립력을 동적 조절하는 연구가 가능합니다.

---

## 3. 연구 방향 2: LLM 기반 로봇 작업 계획 및 자연어 인터랙션

### 3.1 배경 및 동기

대규모 언어모델(LLM)의 로봇 통합은 2023년 이후 가장 활발한 연구 분야 중 하나입니다. LLM이 고수준 언어 명령을 로봇이 실행할 수 있는 동작 시퀀스로 분해하는 능력은 랩스케일에서 특히 유용합니다.

### 3.2 핵심 문헌

**LLM-HRC 프레임워크**: Wakita et al. (2023)은 LLM을 활용하여 고수준 언어 명령을 실행 가능한 동작 함수 시퀀스로 변환하고, YOLO 기반 환경 인지와 결합하여 로봇의 자율적 의사결정을 가능하게 했습니다. LLM의 부정확성은 텔레오퍼레이션과 DMP(Dynamic Movement Primitives)를 통해 보정합니다.

**비파지(Non-prehensile) 도구-물체 조작**: Chen et al. (2026, *Robotics and Computer-Integrated Manufacturing*)은 LLM 기반 기동성 구동(Manoeuvrability-driven) 방법을 제안하여, 로봇이 자연어 지시를 해석하고 도구를 활용한 복잡한 비파지 조작을 수행할 수 있음을 보였습니다.

**VoxPoser / ReKep 등**: 2023~2024년 사이 VoxPoser(3D Value Map 합성), ReKep(관계형 키포인트 제약 추론) 등의 연구가 LLM/VLM과 로봇 조작의 연결 고리를 제시했습니다.

### 3.3 제안 연구 주제

A0509 + AG-145 환경에서 자연어 명령(예: "빨간 비커를 왼쪽 선반으로 옮겨줘")을 YOLO/SAM 기반 물체 인식 → LLM 기반 작업 분해 → MoveIt 기반 경로 계획으로 이어지는 파이프라인을 구축할 수 있습니다. 특히 F/T 센서 없이도 안전한 조작을 위해, LLM이 물체의 재질/무게를 추론하여 그립 전략을 자동 선택하는 시스템이 흥미로운 연구 주제입니다.

---

## 4. 연구 방향 3: 소규모 자율주행 실험실(Self-Driving Lab, SDL)

### 4.1 배경 및 동기

Self-Driving Laboratory는 AI, 로봇, 고성능 컴퓨팅을 결합하여 실험을 자율적으로 설계·수행·분석하는 차세대 연구 패러다임입니다. Nature가 2025년 주목 기술로 선정한 이후 급속도로 성장하고 있으며, 화학·재료과학·생명과학 분야에서 활발히 적용되고 있습니다.

### 4.2 핵심 문헌

**SDL 종합 리뷰**: Tobias & Wahab (2025, *R. Soc. Open Sci.*)는 SDL이 가설 생성부터 실험 설계, 실행, 데이터 분석, 결론 도출까지 과학적 방법 전체를 자동화할 수 있다고 정리했습니다. Cloud Lab을 통한 원격 접근 가능성도 논의됩니다.

**자율 화학 연구**: Song et al. (2025, *J. Am. Chem. Soc.*)의 ChemAgents는 LLM 기반 계층적 다중 에이전트 시스템으로, Task Manager가 Literature Reader, Experiment Designer, Computation Performer, Robot Operator 등 4개의 역할별 에이전트를 조율합니다.

**자율 재료 합성**: Szymanski et al. (2023, *Nature*)의 A-Lab은 ML 기반 합성 레시피 생성, 로봇 고체상 합성, ML 기반 상 식별, 능동학습 최적화를 통합한 자율 재료 발견 플랫폼입니다.

**모듈형 로봇 워크플로**: Dai et al. (2024, *Nature*)는 모바일 로봇이 Chemspeed 합성기, UPLC-MS, 벤치탑 NMR 사이에서 시료를 운반하며 휴리스틱 반응 계획기가 워크플로를 지휘하는 시스템을 제시했습니다.

**SAMPLE 플랫폼**: Romero 팀(2024)은 단백질 공학용 SDL인 SAMPLE(Self-driving Autonomous Machines for Protein Landscape Exploration)에서 Cloud Lab을 통해 원격으로 효소 공학 실험을 수행했습니다.

### 4.3 제안 연구 주제

A0509 + AG-145는 랩스케일 SDL의 핵심 물리적 실행 계층으로 활용할 수 있습니다. 예를 들어 색상 혼합 최적화, 용액 농도 조절 실험 등 비교적 단순한 화학/재료 실험에서 베이지안 최적화(BO) 기반 능동학습과 로봇 파지·운반을 결합한 폐루프 자율 실험 시스템을 구축하는 것이 현실적입니다. Argonne의 Polybot이 9만 개 재료 조합을 몇 주 만에 스크리닝한 사례처럼, 소규모이지만 24/7 운영 가능한 SDL 프로토타입을 목표로 할 수 있습니다.

---

## 5. 연구 방향 4: 모델 기반 Sensorless 충돌 감지 및 안전한 물리적 인간-로봇 상호작용(pHRI)

### 5.1 배경 및 동기

F/T 센서 없이도 안전한 인간-로봇 협업을 실현하는 것은 협동로봇의 비용 절감과 보급에 핵심적인 과제입니다.

### 5.2 핵심 문헌

**적응형 슬라이딩 모드 관측기**: Le & Kang (2022, *Systems Science & Control Engineering*)은 위치 측정만으로 접촉력을 추정하는 적응형 3차 슬라이딩 모드 관측기를 제안했습니다. 이를 어드미턴스 제어와 긴급 정지 제어에 활용하여 센서 없는 pHRI를 실현했습니다.

**신경망 기반 마찰 학습**: 외란 관측기와 신경망 기반 마찰 근사를 결합한 센서리스 힘 추정 방법(2021, *Robotics and Computer-Integrated Manufacturing*)은 산업용 로봇에서의 실용적 적용을 보여주었습니다.

**AI 강화 안전 모니터링**: Tiwari et al. (2025)은 강화학습 기반 시스템으로 로봇이 인간의 움직임 이전에 행동을 조정할 수 있는 예측적 안전 기능을 제안했습니다.

### 5.3 제안 연구 주제

A0509의 관절 토크 데이터를 활용하여, 딥러닝 기반 외란 관측기를 설계하고 이를 통해 엔드이펙터의 접촉력을 실시간 추정하는 연구가 가능합니다. 특히 관절 토크 데이터만으로 파지 중 물체의 미끄러짐(slip)을 감지하거나, 인간과의 의도하지 않은 접촉을 분류하는 연구가 매력적입니다.

---

## 6. 연구 방향 5: Sim-to-Real 전이 학습 기반 조작 기술 습득

### 6.1 배경 및 동기

실제 로봇에서의 시행착오(trial-and-error) 학습은 시간과 비용이 많이 들지만, 시뮬레이션에서 학습한 정책을 실제 로봇으로 전이(Sim-to-Real)하면 효율적으로 조작 기술을 습득할 수 있습니다.

### 6.2 핵심 문헌

**베이지안 최적화와 전이학습**: Ciaravella et al. (2022, *Scientific Reports*)는 PAL Robotics TIAGo에서 언더액추에이티드 컴플라이언트 핸드를 사용하여, 센서리스(힘/토크 측정 불가) 환경에서 베이지안 최적화로 변형 가능한 병의 파지 전략을 학습했습니다. 학습된 지식을 새로운 물체로 전이하는 방법도 제시했습니다.

**디지털 트윈 통합**: 2025년 협동로봇의 주요 트렌드 중 하나로, 디지털 트윈이 로봇의 실세계 성능을 복제하여 위험 없는 가상 환경에서 프로세스를 시뮬레이션·최적화·정제할 수 있게 합니다.

### 6.3 제안 연구 주제

Doosan은 ROS와 Gazebo 시뮬레이션을 공식 지원하므로(GitHub: doosan-robot), A0509 + AG-145의 디지털 트윈을 구축하고, 시뮬레이션에서 강화학습(RL)으로 다양한 물체의 파지 정책을 학습한 뒤 실제 로봇으로 전이하는 파이프라인을 연구할 수 있습니다. 특히 F/T 센서 없이 비전 피드백만으로 파지 성공을 판단하는 보상 함수 설계가 핵심 연구 이슈가 됩니다.

---

## 7. 연구 방향 6: 모듈형 랩 자동화 아키텍처 및 Plug & Play 통합

### 7.1 배경 및 동기

연구 실험실에서의 로봇 통합은 높은 복잡성으로 인해 적응성이 제한됩니다. 표준화된 참조 아키텍처가 필요합니다.

### 7.2 핵심 문헌

**참조 아키텍처 모델**: Fleischer et al. (2024, *SLAS Technology*)는 실험실 워크플로를 계층적으로 분해하고, 자동화 제어 아키텍처의 레이어와 컴포넌트에 매핑하는 참조 아키텍처 모델을 제안했습니다. Pick-and-place 랩웨어 운반을 핵심 지원 단계로 식별하고 상세히 설명했습니다.

**다중 로봇-다중 작업 스케줄링**: Zhou et al. (2025, *Digital Discovery*)는 자율 화학 실험실을 위한 다중 로봇 다중 작업 스케줄링 시스템을 제안했습니다.

**DOE 워크숍 권고사항**: 2024년 Denver에서 열린 "Shaping the Future of Self-Driving Autonomous Laboratories" 워크숍에서는 범용 실험 장비 인터페이스 개발, 자동화된 메타데이터 수집 시스템 구현, 물리적 원리를 이해하는 AI 시스템 개발 등을 핵심 권고사항으로 제시했습니다.

### 7.3 제안 연구 주제

A0509를 중심으로 비커, 피펫, 마이크로플레이트 등 표준 랩웨어의 pick-and-place 자동화를 위한 모듈형 소프트웨어 아키텍처를 설계할 수 있습니다. ROS 기반의 표준화된 "랩웨어 조작 프리미티브" 라이브러리를 구축하고, 새로운 실험 프로토콜이 추가될 때 최소한의 재프로그래밍으로 적응할 수 있는 Plug & Play 시스템을 목표로 하는 것이 현실적인 연구 방향입니다.

---

## 8. 종합 비교 및 우선순위 제안

| 연구 방향 | 기술적 난이도 | 논문 발표 가능성 | 장비 적합도 | 트렌드 부합 |
|-----------|:----------:|:------------:|:---------:|:---------:|
| ① 비전 기반 Sensorless 힘 추정 | ★★★☆ | ★★★★ | ★★★★★ | ★★★★ |
| ② LLM 기반 작업 계획 | ★★★☆ | ★★★★★ | ★★★★ | ★★★★★ |
| ③ 소규모 SDL 프로토타입 | ★★★★ | ★★★★★ | ★★★★ | ★★★★★ |
| ④ Sensorless pHRI 안전 제어 | ★★★☆ | ★★★★ | ★★★★★ | ★★★☆ |
| ⑤ Sim-to-Real 전이학습 | ★★★★ | ★★★★ | ★★★★ | ★★★★ |
| ⑥ 모듈형 랩 자동화 아키텍처 | ★★☆☆ | ★★★☆ | ★★★★ | ★★★★ |

### 추천 전략

**단기 (3~6개월)**: 방향 ①과 ②를 결합하여, LLM 기반 자연어 명령 → 비전 기반 물체 인식 → 적응형 파지 파이프라인을 구축합니다. F/T Sensorless 환경에서의 비전 기반 파지 안정성 모니터링이 차별화 포인트입니다.

**중기 (6~12개월)**: 방향 ③으로 확장하여, 위 파이프라인 위에 베이지안 최적화 기반 자율 실험 루프를 추가합니다. 예를 들어 색상 혼합 최적화나 용액 희석 프로토콜 최적화 등 시각적으로 검증 가능한 실험 태스크에서 소규모 SDL을 시연합니다.

**장기 (12개월 이상)**: 방향 ④, ⑤, ⑥을 종합하여 인간-로봇 협업 가능한 디지털 트윈 기반 적응형 랩 자동화 시스템으로 발전시킵니다.

---

## 9. 주요 참고문헌

1. Xu, Q. et al. (2025). Markerless Force Estimation via SuperPoint-SIFT Fusion and FEA. *Biomimetics*, 10(9), 600.
2. Fu, J. et al. (2024). A Variable Stiffness Robotic Gripper with Vision-Based Force Sensing. *Robotica*, 42(12).
3. Kadalagere Sampath, S. et al. (2025). A Vision-Guided Deep Learning Framework for Dexterous Robotic Grasping. *Applied Sciences*, 15(5), 2615.
4. Le, T. & Kang, H.-J. (2022). Sensorless Contact Force Estimation in Collaborative Robot Based on Adaptive Third-Order Sliding Mode Observer. *Systems Science & Control Engineering*.
5. Tobias, A.V. & Wahab, A. (2025). Autonomous 'Self-Driving' Laboratories: A Review. *R. Soc. Open Sci.*, 12(7), 250646.
6. Song, T. et al. (2025). A Multiagent-Driven Robotic AI Chemist. *J. Am. Chem. Soc.*, 147, 12534-45.
7. Szymanski, N.J. et al. (2023). An Autonomous Laboratory for Accelerated Synthesis of Novel Materials. *Nature*, 624, 86-91.
8. Dai, T. et al. (2024). Autonomous Mobile Robots for Exploratory Synthetic Chemistry. *Nature*, 635, 890-897.
9. Ciaravella, G. et al. (2022). Grasping Learning, Optimization, and Knowledge Transfer. *Scientific Reports*.
10. Tom, G. et al. (2024). Self-Driving Laboratories for Chemistry and Materials Science. *Chem. Rev.*, 124(16), 9633-9732.
11. Hickman, R.J. et al. (2025). Atlas: A Brain for Self-Driving Laboratories. *Digital Discovery*, 4, 1006-1029.
12. Zhou, J. et al. (2025). Multi-Robot Multi-Task Scheduling for Autonomous Chemistry Labs. *Digital Discovery*, 4, 636-652.
13. Wakita, Y. et al. (2023). LLM-Based Human-Robot Collaboration Framework for Manipulation Tasks.
14. Chen, W. et al. (2026). Non-prehensile Tool-Object Manipulation by Integrating LLM-based Planning. *Robotics and Computer-Integrated Manufacturing*.

---

*작성일: 2026년 3월 17일*
*본 보고서는 2024~2025년 발표된 주요 문헌을 기반으로 Doosan A0509 (F/T Sensorless) + DH-Robotics AG-145 환경에서 수행 가능한 연구 방향을 정리한 것입니다.*
