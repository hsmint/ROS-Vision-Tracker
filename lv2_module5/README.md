# Xmen — ROS-Vision-Tracker

RealSense D435와 OpenCR·Dynamixel을 활용한 ROS 2 기반 2축 비전 추적 프로젝트의 보고서와 실험 산출물입니다. 파란 큐브 검출, 중심 오차 기반 제어, 목표 소실 복구 및 bag 재현 결과를 정리합니다.

## 디렉터리 구조

아래 구조는 주요 파일과 폴더를 요약한 것입니다. 반복되는 실험 파일과 이미지 목록은 생략했습니다.

```text
lv2_module5/
├── README.md                         # 프로젝트 구조와 문서 안내
├── report.md                         # 전체 과제 보고서 (문제 1~5, 심화)
├── presentation.md                   # 발표 원고
├── team.md                           # 팀 구성, 역할 및 기여
└── results/
    ├── PERCEPTION.md                 # 인지 튜닝·평가 안내
    ├── answer/                       # 문제별 상세 보고서와 증빙
    │   ├── problem1/
    │   │   ├── report_problem1.md     # 색 기반 검출 결과
    │   │   └── capture/              # 원본·마스크·오버레이 이미지
    │   ├── problem3/
    │   │   ├── report_problem3.md     # P 제어와 Kp 비교
    │   │   ├── gif/                  # 추적 영상
    │   │   └── pic/                  # 제어 흐름 및 결과 그래프
    │   ├── problem4/
    │   │   ├── report_problem4.md     # 성능 측정·소실 복구·정지 시험
    │   │   ├── frames/               # 평가 프레임 및 CSV 기록
    │   │   ├── gif/                  # 시험별 동작 영상
    │   │   └── pic/                  # 상태 흐름 및 결과 그래프
    │   └── problem5/
    │       ├── report_problem5.md     # bag 재현 및 협업 검증
    │       ├── third_party_log.txt    # 다른 팀원의 실행 로그
    │       └── pic/                  # 재현 결과 캡처
    ├── bags/                         # ROS 2 bag 및 분석 결과
    │   ├── kp*_run*/                 # Kp별 반복 실험 (MCAP·metadata.yaml)
    │   ├── p4_ctrl/                  # 제어 통신 중단 시험
    │   ├── p4_input/                 # 인지 입력 중단 시험
    │   ├── p4_normal/                # 정상 추적 시험
    │   ├── p4_occl/                  # 가림·재등장 시험
    │   ├── csv/                      # 시계열 CSV·JSON 및 요약 지표
    │   └── plots/                    # 실험 그래프 및 요약 CSV
    ├── bringup/
    │   ├── TOPICS.md                 # 노드·토픽·QoS·인터페이스 규약
    │   └── parameter.md              # 설정 파라미터 설명
    ├── control/
    │   ├── opencr_control_report.md  # OpenCR 제어 설명
    │   ├── opencr_control_flow.drawio # 편집 가능한 제어 흐름도
    │   ├── opencr_lite_*.png          # 펌웨어 구조·루프·설정 이미지
    │   └── main_control/
    │       ├── main.py               # 제어 Python 코드
    │       └── main_py_report.md      # 코드 설명
    ├── pic/                          # 시스템 및 제어 설명 이미지
    └── presentation/
        ├── presentation.pdf          # 발표 자료 PDF
        └── pic/                      # 발표용 사양·장비·노드 이미지
```

## 주요 문서

| 문서 | 내용 |
| --- | --- |
| [전체 보고서](report.md) | 시스템 구성부터 문제 1~5의 구현·측정·해석까지 |
| [발표 원고](presentation.md) · [발표 PDF](results/presentation/presentation.pdf) | 문제 정의, 인지·제어, 결과와 배운 점 |
| [팀 구성과 기여](team.md) | 팀원별 역할과 구현·협업 기록 |
| [인지 튜닝·평가](results/PERCEPTION.md) | 검출기 실행과 튜닝·평가 도구 안내 |
| [토픽·인터페이스](results/bringup/TOPICS.md) | 문제 2 관련 노드 연결, 메시지 규약, QoS 및 정지 규칙 |
| [파라미터 설명](results/bringup/parameter.md) | 추적·제어 설정의 의미 |
| [OpenCR 제어](results/control/opencr_control_report.md) | 하드웨어 제어 흐름 설명 |
| [Python 제어 코드 설명](results/control/main_control/main_py_report.md) | `main.py`의 구조와 동작 |

## 문제별 결과

| 문제 | 상세 자료 |
| --- | --- |
| 1 — 색 기반 검출 | [보고서](results/answer/problem1/report_problem1.md) 및 원본·마스크·오버레이 캡처 |
| 2 — 인터페이스 규약 | [전체 보고서](report.md)와 [토픽 문서](results/bringup/TOPICS.md) |
| 3 — 중심 기반 추적 제어 | [보고서](results/answer/problem3/report_problem3.md), Kp별 그래프와 GIF |
| 4 — 성능 측정과 소실 복구 | [보고서](results/answer/problem4/report_problem4.md), 평가 프레임과 시험 영상 |
| 5 — bag 재현과 팀 협업 | [보고서](results/answer/problem5/report_problem5.md), 재현 캡처와 실행 로그 |

`results/bags/`의 원본 기록과 `csv/`, `plots/`의 분석 결과를 함께 확인하면 보고서의 측정 근거를 추적할 수 있습니다. 전체 시스템 실행에 관한 명령과 환경 설명은 각 문서를 참고하세요.
