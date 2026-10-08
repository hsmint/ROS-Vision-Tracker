# F팀 Xmen 팀 구성과 기여

> 프로젝트명: ROS-Vision-Tracker  
> 대상: [홍석민](https://github.com/hsmint) · [김혜민](https://github.com/heffeekim94-web) · [최형준](https://github.com/chj1319) · [이홍주](https://github.com/kanichong)  

## 1. 팀 구성과 역할 분담

### 1.1 책임 매트릭스

| 팀원 | GitHub 계정 | 주 역할 | 구체적인 담당 범위 | 협업 접점 |
| --- | --- | --- | --- | --- |
| [홍석민](https://github.com/hsmint) | `hsmint` | 팀장·제어·통합 | 저장소 설정, ROS–OpenCR 연동, URDF 저장소 반영·launch 연동, tracker 구조 통합, lite 제어 전환 | 팀 운영, 기능 통합, PR 병합과 공식 리뷰 |
| [김혜민](https://github.com/heffeekim94-web) | `heffeekim94-web` | 제어 | 모터 구동, 펌웨어·Python 명령, X/Y축 동시 입력·공통 속도, URDF 모델 제작, 코드·설명 자료 정리 | 인지 측 입력 형식 조정, ROS 제어 연결을 위한 초기 제어 구현 |
| [최형준](https://github.com/chj1319) | `chj1319` | ROS 통합 | 토픽·QoS, 초기 인지·제어 파이프라인, launch·검증 노드, OpenCR 시리얼·joint_states | 인지 데이터와 제어 명령 연결, 역할별 패키지·실행 구조 정리 |
| [이홍주](https://github.com/kanichong) | `kanichong` | 인지·추적 | HSV·Contour·depth·형상 검증, 실측 튜닝·평가, cube detector, CSRT·단절 복구 | 객체 검출·추적 결과를 tracker·제어 흐름과 연결 |
| 전원 | 해당 없음 | 공동 검증 | 테스트 및 검증 | 기능 간 연동과 실제 환경 동작 확인 |

---

## 2. 팀원별 상세 기여

### 2.1 [홍석민](https://github.com/hsmint)

#### 담당 역할

팀장과 제어를 맡아 저장소를 구성하고, 이후 ROS–OpenCR 연동, URDF의 저장소 반영과 시각화·launch 연동, tracker 통합, lite 제어 전환을 수행했다.

#### 주요 기여

- **팀 운영:** 초기 저장소 설정을 완료하고 각 기능의 변경을 병합했다.
- **제어 연결:** `/cmd_vel`의 `Twist`와 OpenCR 통신을 연결하고 위치 읽기를 지원했다.
- **실행·시각화 통합:** 제작된 URDF를 저장소에 반영하고 시각화·launch 구성을 추가했다.
- **구조 통합:** 인지 기능을 `xmen_tracker`로 모으고 기본 제어를 lite 경로로 전환했다.
- **검토·병합:** PR #7·#8을 작성하고 팀 PR 8건을 병합했다.

#### 구현과 협업

| 구분 | 수행 내용 | 산출물 |
| --- | --- | --- |
| ROS와 제어 연결 | `/cmd_vel`의 `Twist`를 OpenCR에 전달하고 위치를 읽는 기능을 구현했다 | [작성 커밋 c45ef0a2](https://github.com/hsmint/ROS-Vision-Tracker/commit/c45ef0a297fd6e3609f6fd93fe2e0c8931f3e224), [PR #7](https://github.com/hsmint/ROS-Vision-Tracker/pull/7) |
| 모델 반영·실행 통합 | 제작된 URDF를 저장소에 반영하고 시각화·launch 구성을 추가했다 | [작성 커밋 acf598f5](https://github.com/hsmint/ROS-Vision-Tracker/commit/acf598f526843ed911f9f17c4beee53c6dcec0d0), [PR #8](https://github.com/hsmint/ROS-Vision-Tracker/pull/8) |
| 통합 구조 정리 | tracker 인지 기능을 `xmen_tracker`로 통합했다 | [작성 커밋 aa276ea4](https://github.com/hsmint/ROS-Vision-Tracker/commit/aa276ea49416f9fd2e690d74fac3d84b931e4169) |
| 제어 경량화와 개선 | OpenCR lite 버전을 추가하고 기본 제어를 lite로 전환했다. 모터 제한과 시간 지연 관련 코드를 수정했다 | [0c6320fd](https://github.com/hsmint/ROS-Vision-Tracker/commit/0c6320fd22a770742921ca239c1612a11d93ceef), [01051a0a](https://github.com/hsmint/ROS-Vision-Tracker/commit/01051a0a014810df25bf323d378ec02580c4827c), [c24f14dc](https://github.com/hsmint/ROS-Vision-Tracker/commit/c24f14dc1d952e9366fd40ae0a427f8fab23c1c4) |

---

### 2.2 [김혜민](https://github.com/heffeekim94-web)

#### 담당 역할

제어 영역의 모터 구동 시연과 `.ino`·`.py` 코드 작성, 팬·틸트 구조의 URDF 모델 제작을 맡았다. 초기 펌웨어를 적용하고 X/Y축 동시 명령, 속도·각도 설정을 구현하고 인지 측과 입력 형식을 조정했으며, 부팅 시 위치 복원을 조사했다.

#### 날짜별 수행 내용

| 날짜 | 작업 | 상태 | 수행 내용과 결과 | 산출물 |
| --- | --- | --- | --- | --- |
| 10월 1일 | 펌웨어 업데이트와 모터 확인 | 완료 | 펌웨어를 적용하고 라즈베리파이에서 모터 구동과 통신을 검증했다. 초기 제어 코드는 10월 2일 커밋했다 | [초기 제어 커밋](https://github.com/hsmint/ROS-Vision-Tracker/commit/c0d8332e14aa9e1327fee5df5c819834483e13eb), [PR #2](https://github.com/hsmint/ROS-Vision-Tracker/pull/2) |
| 10월 3일 | 입력값·최대값 수정, ROS 2 구독자 노드 생성 | 완료 | X/Y축 목표각을 한 입력으로 받고 두 모터에 속도를 동시에 적용했다. 이날 모터 시험에서 최대 속도 274 deg/s, 최대 각도 ±180°를 확인했다 | [동시 명령 커밋](https://github.com/hsmint/ROS-Vision-Tracker/commit/1452344a198f9f2cca26eca8b2200b50413f612f), [PR #2](https://github.com/hsmint/ROS-Vision-Tracker/pull/2) |
| 10월 6일 | 파일 가독성·이름 정리, IMU 위치값 관련 작업 | 완료 | `auto on/off` 기능을 제거하고 인지 담당 측의 입력 수정을 연계했다. 5배 가속력에 따른 반동을 분석하고 통합 측 가속력 재설정 필요성을 정리했다. URDF 모델 제작과 문제 리뷰를 완료했다 | [코드 정리 커밋](https://github.com/hsmint/ROS-Vision-Tracker/commit/9e42ad5265c0c768202276b70970d5eb8ed88cb2), [PR #3](https://github.com/hsmint/ROS-Vision-Tracker/pull/3) |

#### 문제 해결 과정

1. **설치·실행 환경 정리**
   - Arduino CLI, OpenCR 코어 1.5.3, 라이브러리와 빌드·업로더·포트 확인 절차를 정리했다.
   - SSH 대상 장치에 필요한 설치를 PC 터미널에서 실행한 문제를 파악하고 작업 환경을 구분했다.
   - 기존 `opencr_position_p.ino`를 수정하고 모터 ID를 확인했다.
2. **두 축 동시 명령 구현**
   - X/Y축 목표각과 공통 속도를 한 번에 입력하고 두 모터에 동시에 적용하도록 수정했다.
   - 10월 3일 모터 시험에서 속도·각도 범위를 확인했다. 당시 시험값은 274 deg/s와 ±180°다.
3. **부팅 시 위치 복원과 반동 분석**
   - `auto on/off` 설정, 마지막 위치 복원과 틱 단위 파라미터를 조사한 뒤 `auto on/off` 기능을 제거했다.
   - 전원 연결 시 회전 반동의 원인을 5배 가속력으로 분석하고, 통합 측에서 가속력을 재설정하도록 작업 내용을 정리했다.
   - 10월 6일 URDF 모델 제작과 제어 코드 정리를 완료했다. 부팅 반동 해소 여부는 통합 장비 시험에서 추가 검증이 필요하다.

#### 구현과 문서 정리

| 구분 | 수행 내용 | 산출물 |
| --- | --- | --- |
| 초기 제어 구현 | 카메라 제어 코드를 작성하고 진행 자료를 정리했다 | [c0d8332e](https://github.com/hsmint/ROS-Vision-Tracker/commit/c0d8332e14aa9e1327fee5df5c819834483e13eb) |
| 두 축 동시 제어 | X/Y 움직임 명령을 동시에 적용하도록 control 패키지를 수정했다 | [1452344a](https://github.com/hsmint/ROS-Vision-Tracker/commit/1452344a198f9f2cca26eca8b2200b50413f612f), [83a5479c](https://github.com/hsmint/ROS-Vision-Tracker/commit/83a5479cfb33cd3f9bcb84479e5a1bfa286c6163), [PR #2](https://github.com/hsmint/ROS-Vision-Tracker/pull/2) |
| URDF 모델 제작 | 팬·틸트 구조의 URDF 모델을 제작했다 | 10월 6일 제작 완료, [URDF 산출물](https://github.com/hsmint/ROS-Vision-Tracker/blob/8432524e0d5fd8cf6cee40536bbffdeb1c50b934/xmen_description/urdf/cctv.urdf). 저장소 반영·launch 연동은 [홍석민](https://github.com/hsmint)이 담당했다 |
| 정리와 수정 | 코드 가독성을 개선하고 `mian.py`를 `main.py`로 수정했다. 그림 폴더와 제어 설명 자료를 보완했다 | [9e42ad52](https://github.com/hsmint/ROS-Vision-Tracker/commit/9e42ad5265c0c768202276b70970d5eb8ed88cb2), [bcb5f547](https://github.com/hsmint/ROS-Vision-Tracker/commit/bcb5f5477ba391de62942576f13dd49c11bdbf10), [PR #3](https://github.com/hsmint/ROS-Vision-Tracker/pull/3) |

작성한 PR #2·#3이 병합되었다. #2는 develop에 직접 병합되었고, #3은 control에 병합된 뒤 #7을 통해 develop에 포함되었다. develop의 8개 커밋 중 일반 7개, merge 1개다.

---

### 2.3 [최형준](https://github.com/chj1319)

#### 담당 역할

통합 노드와 구동 테스트를 맡아 인지·제어 작업을 지원하고 ROS 통합 구조를 설계했다. 라즈베리파이의 RealSense 입력, 토픽 주기와 QoS를 다뤘다.

#### 날짜별 수행 내용

| 날짜 | 작업 내용 | 상태 | 수행 내용과 결과 |
| --- | --- | --- | --- |
| 10월 1일 | 통합 영역의 펌웨어 업데이트 | 완료 | 통합 영역의 펌웨어 업데이트를 완료했다 |
| 10월 2일 | depth 데이터를 이용한 퍽 크기 확인, 인지 ROS 노드 구성 | 완료 | depth로 퍽 크기를 검증하고 인지 ROS 노드를 구성했다. RealSense 입력·RGBD 구독, HSV·Contour·depth 검사, `/target`, 토픽·QoS와 launch 관련 코드는 10월 3일 커밋했다 |
| 10월 6일 | 역할별 폴더 정리, 명령 토픽을 `cmd_vel`로 변경, 문서 정리 | 일지: 시작 전 / 코드: 반영 완료 | 역할별 폴더와 문서를 정리하고 명령 토픽을 `/cmd_vel`로 변경했다. PR #5를 통해 develop에 반영했다 |

#### 구현과 검증

| 구분 | 수행 내용 | 산출물 |
| --- | --- | --- |
| 인터페이스 | `/search`와 토픽·QoS의 공통 정의를 구성하고 명령 토픽을 `/cmd_vel`로 변경했다 | [a363b2af](https://github.com/hsmint/ROS-Vision-Tracker/commit/a363b2afebb6445905578969a5b843a0d3695a0a), [5cc8d389](https://github.com/hsmint/ROS-Vision-Tracker/commit/5cc8d3896097f7d317035890faaddb9df4d79e8b) |
| 초기 인지 통합 | RealSense 직접 입력·RGBD 구독, HSV·Contour·depth 검증과 `/target` 발행을 구현했다 | [14c59901](https://github.com/hsmint/ROS-Vision-Tracker/commit/14c5990117b6c1cd6a787973aba927879a4ee708) |
| 실행·검증 | 전체 실행 launch와 설정, 검증 노드·launch를 작성하고 실행·인터페이스 문서를 정리했다 | [e65647b6](https://github.com/hsmint/ROS-Vision-Tracker/commit/e65647b612cda3cb973857e3420fc56bac5510de), [ea1cd634](https://github.com/hsmint/ROS-Vision-Tracker/commit/ea1cd6342651d4d61a932452ff2424074158c657) |
| 제어·시리얼 | 팬·틸트 제어, watchdog·출력 OFF를 구현했다. OpenCR V 명령 전송·P 응답 수신, `/joint_states` 발행과 회전 제한을 추가했다 | [50c2408b](https://github.com/hsmint/ROS-Vision-Tracker/commit/50c2408b5a4e0112e23c8057f6c93771262561be), [296c9869](https://github.com/hsmint/ROS-Vision-Tracker/commit/296c9869294fd65ca2f4d2430daa7594a1f8b4fd) |
| 패키지 정리 | 인지·제어·bringup을 역할별 폴더로 옮기고 카메라 입력 방식 선택 기능을 추가했다 | [bd494262](https://github.com/hsmint/ROS-Vision-Tracker/commit/bd494262f9163058579e7ee4a2e4ee0e81135b57) |
| 병합 협업 | [홍석민](https://github.com/hsmint)이 작성한 PR #8을 병합했다 | [PR #8](https://github.com/hsmint/ROS-Vision-Tracker/pull/8) |

PR #5를 작성했으며, develop에 포함된 커밋 12개 중 일반 커밋은 11개, merge 커밋은 1개다. PR #5에서 보고한 시험 결과는 다음과 같다.

- 모의 입력 시험 11/11 통과
- 모터 출력 OFF 조건에서 `/search` 시험 5/5 통과
- 실제 RPi 카메라 입력에서 `/target` 29.7 Hz, `/cmd_vel` 20 Hz 측정
- 가상 시리얼에서 명령 전송·끊김·제한 동작 검증

초기 `xmen_vision` 인지 노드와 ROS 인터페이스를 구현했다. 이후 통합 리팩터링으로 인지 코드는 `xmen_tracker`에 통합되었다.

---

### 2.4 [이홍주](https://github.com/kanichong)

#### 담당 역할

인지 영역의 카메라 입력, HSV·Contour 기반 검출, 객체 중심 계산과 미검출 처리를 맡았다. 거리·형상·깊이 검증을 구현하고, 조명 변화에 맞춰 검출 조건을 조정하며 성능을 평가했다.

#### 날짜별 수행 내용

| 날짜 | 작업 | 상태 | 수행 내용과 결과 | 산출물 |
| --- | --- | --- | --- | --- |
| 10월 1일 | 인지 예제 확인과 환경 구성 | 완료 | Raspberry Pi 펌웨어와 Ubuntu Server 24.04 이미지를 준비하고 SSH 연결·26.04 업그레이드를 진행했다. OpenCV mask·HSV·중심점 예제를 분석하고 RealSense 예제 동작을 확인했다 | 일지, [커밋 링크](https://github.com/hsmint/ROS-Vision-Tracker/commit/a251858086fe290097ee5971d8fb683cd8bdf2bf) |
| 10월 2일 | 파란 큐브 마스크·검출·추적 | 완료 | 0.1·0.2·0.5·1.0 m 거리와 가림·사라짐 조건에서 HSV·Contour 검출과 객체 중심 추적을 검토했다. 객체 사각형과 컨투어 출력을 확인했다 | 일지, [커밋 링크](https://github.com/hsmint/ROS-Vision-Tracker/commit/2a7c5d4bfe9406e9b42c6626f6055ce4570c01d2) |
| 10월 6일 | 실제 환경 HSV 튜닝과 CSRT 적용 검토 | 진행 중 | 조명·거리별 데이터로 기존 설정과 최종 설정을 비교하고 HSV 값을 수정했다. CSRT 적용을 검토했다 | 일지, [커밋 링크](https://github.com/hsmint/ROS-Vision-Tracker/commit/cc1395caa5556a392c0ec684dcdba40bddef0b77) |

#### 검출 조건과 접근 방법

10월 2일에는 다음 조건으로 검출과 추적을 검토했다.

- 실측 HSV 범위: H 104~120, S 200 이상
- 장단비, 채움비, solidity, `box_fit` 검사
- 거리 0.1~1.1 m 확인
- 실제 크기: 짧은 변 6 cm 이하, 긴 변 8 cm 이하 확인
- 가림·사라짐 조건과 객체 중심 추적 검토

10월 6일에는 어두운 배경의 옅은 파랑이 큐브 마스크에 붙으면서 크기가 과대 계산되고 `size_mismatch`로 탈락하는 문제를 분석했다. 큐브만 남도록 마스크를 조정해 평가 데이터의 검출률과 처리 시간을 개선했으며, 실측 보정을 PR #6에 반영했다. 이후 CSRT 보완과 영상 단절 복구를 구현해 PR #13에 반영했다.

#### 조건별 성능 평가

동일한 데이터에 최적화 전·최종 설정을 적용해 조건당 100장으로 평가했다. 아래는 당시 평가 결과다. 처리 시간은 해상도 맞춤부터 색·모양·깊이 검증과 중심 계산까지 측정했으며, 화면 표시와 카메라 대기 시간은 제외했다.

| 조건 | 최적화 전 검출률 | 최종 검출률 | 최적화 전 처리 ms 평균 / p95 | 최종 처리 ms 평균 / p95 |
| --- | --- | --- | --- | --- |
| 어두움, 약 0.5 m | 0% | 100% | 7.58 / 9.36 | 1.03 / 1.26 |
| 보통, 약 0.5 m | 100% | 100% | 6.07 / 6.62 | 0.96 / 1.15 |
| 밝음, 약 0.5 m | 100% | 100% | 2.84 / 3.03 | 0.97 / 1.12 |
| 0.1 m 조건, 실측 Z 0.16 m | 100% | 100% | 3.19 / 3.36 | 1.03 / 1.18 |
| 0.2 m 조건, 실측 Z 0.32 m | 100% | 100% | 6.05 / 6.52 | 1.03 / 1.17 |
| 0.5 m 조건, 실측 Z 0.55 m | 100% | 100% | 6.07 / 6.62 | 0.96 / 1.15 |
| 어두움, 1.0 m 조건, 실측 Z 1.02 m | 0% | 100% | 7.04 / 8.82 | 1.28 / 1.46 |
| 밝음, 1.0 m 조건, 실측 Z 1.02 m | 3% | 100% | 5.42 / 5.90 | 0.96 / 1.11 |

- 평가 요약 기준 평균 처리 시간을 **5.3 → 1.0 ms**로 줄였다.
- 보통 0.5 m는 조명·거리 평가에 공통으로 사용한 조건이다.
- 실제 카메라 100프레임 측정에서는 프레임 대기 26.4 ms, 검출 처리 약 1 ms를 측정했다. 위 표의 처리 시간은 검출 단계의 시간이다.
- 100% 검출률은 표에 제시한 조명·거리별 평가 데이터에서 얻은 결과다.
- 10월 6일 CSRT 적용 검토 후, 검출 실패 보완·현재 프레임 색/깊이 재검증·단절 복구를 구현했다. 최신 통합본의 재평가는 남아 있다.

#### 인지와 추적 구현

| 구분 | 수행 내용 | 산출물 |
| --- | --- | --- |
| 깊이·초점거리 | 카메라 정보와 depth·형상·HSV 설정을 보정했다 | [a2518580](https://github.com/hsmint/ROS-Vision-Tracker/commit/a251858086fe290097ee5971d8fb683cd8bdf2bf), [PR #4](https://github.com/hsmint/ROS-Vision-Tracker/pull/4) |
| 튜닝·평가 | 큐브 검출을 튜닝하고 evaluate를 추가했다. 실습실 밝기·위치·깊이를 실측해 보정했다 | [2a7c5d4b](https://github.com/hsmint/ROS-Vision-Tracker/commit/2a7c5d4bfe9406e9b42c6626f6055ce4570c01d2), [cc1395ca](https://github.com/hsmint/ROS-Vision-Tracker/commit/cc1395caa5556a392c0ec684dcdba40bddef0b77), [PR #6](https://github.com/hsmint/ROS-Vision-Tracker/pull/6) |
| detector 통합 | 실측 HSV와 실제 크기·모양 검사를 적용하고 일부 가려진 큐브를 처리하도록 detector를 구현해 tracker에 연결했다 | [64aadda4](https://github.com/hsmint/ROS-Vision-Tracker/commit/64aadda4a545a4678808d85e5b19541cda462c65), [PR #10](https://github.com/hsmint/ROS-Vision-Tracker/pull/10) |
| 처리 지연 보완 | 후보마다 전체 영상의 깊이를 계산하던 부분을 후보 주변 ROI 계산으로 개선했다 | [PR #11](https://github.com/hsmint/ROS-Vision-Tracker/pull/11) |
| CSRT와 복구 | 검출 실패 시 CSRT 보완과 현재 프레임 색·깊이 재검증을 구현했다. 같은 계열 색의 배경을 분리하고 영상 단절 후 추적 상태를 초기화하도록 수정했다 | [a1972961](https://github.com/hsmint/ROS-Vision-Tracker/commit/a1972961fb0d4359c752ec4ca66ac46a6100a55f), [PR #13](https://github.com/hsmint/ROS-Vision-Tracker/pull/13) |
| 병합 협업 | [홍석민](https://github.com/hsmint)의 ROS 제어 통합 PR #7을 병합했다 | [PR #7](https://github.com/hsmint/ROS-Vision-Tracker/pull/7) |

---

## 3. PR과 코드 산출물

### 3.1 코드 산출물

코드 산출물은 [ROS-Vision-Tracker 저장소](https://github.com/hsmint/ROS-Vision-Tracker)의 [develop 기준 커밋](https://github.com/hsmint/ROS-Vision-Tracker/commit/8432524e0d5fd8cf6cee40536bbffdeb1c50b934)에 반영되었다.

| 영역 | 주요 산출물 | 내용 |
| --- | --- | --- |
| 인지·추적 | `xmen_tracker` | 카메라 입력, HSV·Contour·깊이 검증, 큐브 추적, 목표와 제어 명령 발행 |
| 제어 | `xmen_control`, `firmware` | ROS 명령과 OpenCR 시리얼 연결, 모터 구동, 홈 복귀, 오류·timeout 처리 |
| 통합 실행 | `xmen_bringup` | 하드웨어 통합 launch, bag 입력, preview·RViz 실행 |
| 모델 | `xmen_description` | 팬·틸트 URDF와 시각화 자료. URDF 제작은 [김혜민](https://github.com/heffeekim94-web), 저장소 반영·시각화·launch 연동은 [홍석민](https://github.com/hsmint) 담당 |
| 팀 공유 자료 | 라즈베리 셋업 가이드, 인지 코드 압축파일, 조립 모델 | 라즈베리파이 환경 설정, 인지 코드와 조립 구조를 Notion 팀 페이지에 공유했다 |

### 3.2 PR 일자

총 **10건의 PR을 병합했다.** 작성자는 PR 개설자이며, 날짜는 UTC 기준이다.

| PR | 작성자 | 생성일 → 병합일 | 대상과 상태 | 실제 병합자 | 핵심 내용 |
| --- | --- | --- | --- | --- | --- |
| [#2](https://github.com/hsmint/ROS-Vision-Tracker/pull/2) | [김혜민](https://github.com/heffeekim94-web) | 2026-10-03 → 2026-10-04 | `control` → `develop`; 병합 | [홍석민](https://github.com/hsmint) | 두 축 동시 입력·공통 속도·제어 패키지 |
| [#3](https://github.com/hsmint/ROS-Vision-Tracker/pull/3) | [김혜민](https://github.com/heffeekim94-web) | 2026-10-06 → 2026-10-06 | `control` → `control`; 병합 | [홍석민](https://github.com/hsmint) | 펌웨어 정리·Python 파일명·Markdown 자료 |
| [#4](https://github.com/hsmint/ROS-Vision-Tracker/pull/4) | [이홍주](https://github.com/kanichong) | 2026-10-06 → 2026-10-06 | `vision` → `develop`; 병합 | [홍석민](https://github.com/hsmint) | depth·초점거리·HSV·형상 조건 |
| [#5](https://github.com/hsmint/ROS-Vision-Tracker/pull/5) | [최형준](https://github.com/chj1319) | 2026-10-06 → 2026-10-06 | `integration` → `develop`; 병합 | [홍석민](https://github.com/hsmint) | ROS 추적 파이프라인·역할별 패키지·검증 |
| [#6](https://github.com/hsmint/ROS-Vision-Tracker/pull/6) | [이홍주](https://github.com/kanichong) | 2026-10-06 → 2026-10-06 | `vision` → `develop`; 병합 | [홍석민](https://github.com/hsmint) | 실습실 밝기·거리·depth 실측 튜닝 |
| [#7](https://github.com/hsmint/ROS-Vision-Tracker/pull/7) | [홍석민](https://github.com/hsmint) | 2026-10-06 → 2026-10-06 | `control` → `develop`; 병합 | [이홍주](https://github.com/kanichong) | ROS–OpenCR 제어 통신·위치 읽기 |
| [#8](https://github.com/hsmint/ROS-Vision-Tracker/pull/8) | [홍석민](https://github.com/hsmint) | 2026-10-06 → 2026-10-06 | `integration` → `develop`; 병합 | [최형준](https://github.com/chj1319) | [김혜민](https://github.com/heffeekim94-web) 제작 URDF의 저장소 반영·시각화·실행 launch |
| [#10](https://github.com/hsmint/ROS-Vision-Tracker/pull/10) | [이홍주](https://github.com/kanichong) | 2026-10-07 → 2026-10-07 | `feature/tracker-detector` → `develop`; 병합 | [홍석민](https://github.com/hsmint) | cube detector를 develop에 반영 |
| [#11](https://github.com/hsmint/ROS-Vision-Tracker/pull/11) | [이홍주](https://github.com/kanichong) | 2026-10-08 → 2026-10-08 | `vision` → `develop`; 병합 | [홍석민](https://github.com/hsmint) | 인지 구조·ROI 깊이 계산·CSRT 보완 |
| [#13](https://github.com/hsmint/ROS-Vision-Tracker/pull/13) | [이홍주](https://github.com/kanichong) | 2026-10-08 → 2026-10-08 | `vision` → `develop`; 병합 | [홍석민](https://github.com/hsmint) | cube tracker·CSRT·배경 분리·단절 복구 |

- #3은 fork의 control → 원본 control 병합이며, 이후 #7을 통해 해당 커밋들이 develop에 포함되었다.
- #4는 [최형준](https://github.com/chj1319)·[이홍주](https://github.com/kanichong), #5는 [최형준](https://github.com/chj1319)·[홍석민](https://github.com/hsmint), #7은 [김혜민](https://github.com/heffeekim94-web)·[홍석민](https://github.com/hsmint), #11은 [이홍주](https://github.com/kanichong)·[홍석민](https://github.com/hsmint)이 작성한 커밋을 함께 포함한다.

### 3.3 기능별 통합 결과

`RealSense → realsense_node → tracker_node(detector + CubeTracker + P 제어) → /cmd_vel → control_lite → opencr_lite → 팬·틸트 모터`

| 영역 | develop 통합 결과 | 담당 기여 |
| --- | --- | --- |
| 카메라 입력 | BGR8 영상과 색상 정렬 depth를 같은 타임스탬프로 발행, CameraInfo 제공. 최신 RGB/depth 쌍만 처리 | [최형준](https://github.com/chj1319)이 초기 ROS 입력과 인터페이스를 구현했다 |
| 객체 인지 | HSV·Contour·모양·depth·실제 크기 검사. 잘린/가려진 후보와 붙은 배경 분리 | [이홍주](https://github.com/kanichong)가 튜닝·detector·추적 기능을, [최형준](https://github.com/chj1319)이 초기 인지 노드를 구현했다 |
| 시간적 추적 | SEARCH/CONFIRM/TRACK, 검출 실패 시 CSRT 보완 후 현재 프레임 색·깊이 검증 | [이홍주](https://github.com/kanichong)가 CSRT 보완과 현재 프레임 재검증을 구현하고 PR #13에 반영했다 |
| 목표와 명령 | `/target`의 x/y는 정규화 영상 오차, z는 면적 비율. `/cmd_vel`은 pan=`angular.z`, tilt=`angular.y`, rad/s | P 제어에 출력 제한을 적용하고, 미검출·오래된 입력에서 정지하도록 구현했다 |
| 모터 제어 | 기본 경로는 `control_lite`와 `opencr_lite.ino`. 피드백 기반 홈 복귀, 시리얼 명령/피드백 유효성 검사, timeout·fault 처리 | [김혜민](https://github.com/heffeekim94-web)이 초기 제어를 구현하고 [홍석민](https://github.com/hsmint)이 ROS 연동과 lite 전환을 수행했다 |
| 실행·시각화 | hardware launch, bag 입력 시 모터 제어 비활성화, 독립 preview·RViz 노드, URDF 모델 | [최형준](https://github.com/chj1319)이 초기 launch·검증 구성을, [김혜민](https://github.com/heffeekim94-web)이 URDF 제작을, [홍석민](https://github.com/hsmint)이 시각화·launch 연동을 담당했다 |

기본 제어 경로의 [lite 펌웨어](https://github.com/hsmint/ROS-Vision-Tracker/blob/8432524e0d5fd8cf6cee40536bbffdeb1c50b934/firmware/opencr_lite.ino)는 pan ID11·tilt ID12를 사용하며, 각도 제한은 pan ±180°, tilt 약 ±120°다. 기본 bridge의 명령·피드백 timeout은 0.2초다. 별도의 full 경로(`control.py` + `opencr_control.ino`)는 EEPROM 복원과 tilt ±135° 제한을 포함하며 프로토콜·timeout 설정도 다르다. 실제 업로드 펌웨어와 [기본 launch](https://github.com/hsmint/ROS-Vision-Tracker/blob/8432524e0d5fd8cf6cee40536bbffdeb1c50b934/xmen_bringup/launch/hardware_launch.py)의 조합, 홈·회전 방향·단절 시 정지는 장비 검증이 남아 있다.

**실물 피드백과 시각화:** JointState 이름은 `pan`/`tilt`, [URDF](https://github.com/hsmint/ROS-Vision-Tracker/blob/8432524e0d5fd8cf6cee40536bbffdeb1c50b934/xmen_description/urdf/cctv.urdf#L520-L549)의 가동 관절명은 `revolute_1`/`revolute_2`이며 각도 제한도 lite 펌웨어와 다르다. [RViz launch](https://github.com/hsmint/ROS-Vision-Tracker/blob/8432524e0d5fd8cf6cee40536bbffdeb1c50b934/xmen_bringup/launch/rviz2_launch.py)는 관절명 변환 없이 URDF를 사용하고 기본값으로 합성 joint state를 발행한다. 실물 피드백과 모델 동작을 맞추기 위한 관절명·제한·발행 설정 점검과 장비 재검증이 필요하다.

---

## 4. 리뷰와 협업

### 4.1 공식 코드 리뷰

[홍석민](https://github.com/hsmint)이 PR #13을 검토하고 승인했다. 병합 PR 10건에 제출된 팀원의 공식 코드 리뷰는 **1건**이다.

| PR | 리뷰어 | 제출 시각 UTC | 공식 상태 | 근거 |
| --- | --- | --- | --- | --- |
| [#13](https://github.com/hsmint/ROS-Vision-Tracker/pull/13) | [홍석민](https://github.com/hsmint) (`hsmint`) | 2026-10-08 01:36:14 | APPROVED | [공식 리뷰 제출 기록](https://github.com/hsmint/ROS-Vision-Tracker/pull/13#pullrequestreview-5450389248) |

PR #2·#7·#8의 Copilot 항목은 할당량 초과 통지로, 코드 리뷰를 수행하지 못했다.

### 4.2 협업 방식

1. **역할 분담:** 10월 1일 회의에서 팀장·제어·통합·인지를 나누고 저장소 설정, 펌웨어, ROS 통합과 비전 최적화 작업을 배정했다.
2. **진행 상황 공유:** 타임라인과 개인 일지에 담당 작업, 문제 해결 과정과 산출물을 정리해 공유했다.
3. **인터페이스 조정:** 인지 측 입력 형식을 통일하고 제어 명령 토픽을 `/cmd_vel`로 변경했다.
4. **기능별 검증:** [이홍주](https://github.com/kanichong)가 조명·거리별 인지 성능을 평가하고 [김혜민](https://github.com/heffeekim94-web)이 모터 구동·통신과 두 축 동작을 검증했다.
5. **코드 통합:** 기능별 PR을 병합해 인지·추적·제어·실행 코드를 develop에 통합했다.

### 4.3 남은 통합 검증

같은 develop 버전으로 다음 항목을 검증해야 한다. 관련 기여자는 아래와 같으며, 시험 담당·일정·성공 기준은 미정이다.

| 영역 | 관련 기여자 | 남은 검증 항목 |
| --- | --- | --- |
| 제어·안전 | [홍석민](https://github.com/hsmint)·[김혜민](https://github.com/heffeekim94-web)의 제어 작업 | 업로드 펌웨어와 full/lite 조합, pan ID11·tilt ID12, 홈 순서·회전 부호·기구 제한, 명령·피드백 단절과 오류 후 복구를 확인. 장비·펌웨어·입력·실제 위치/상태 로그를 함께 기록 |
| 통합·시각화 | [최형준](https://github.com/chj1319)의 ROS 통합, [김혜민](https://github.com/heffeekim94-web)의 URDF 제작, [홍석민](https://github.com/hsmint)의 launch 연동 | 현재 develop 빌드·실행, RGB/depth/CameraInfo·토픽 주기·QoS·명령 흐름을 확인. 관절명·축·영점·제한과 RViz 합성 joint-state 설정을 점검하고 실측 피드백과 모델 동작을 로그/영상으로 대조 |
| 인지·추적 | [이홍주](https://github.com/kanichong)의 튜닝·detector·CSRT 구현 | 조명·거리·같은 색 배경·부분 가림별 검출/오검출과 처리 시간을 재평가. 후보 수 증가 시 지연, 미검출 보완, 영상 단절 후 초기화·재획득을 같은 장비·영상·설정으로 확인 |

검출·추적 보완 코드는 병합을 완료했으며, 현재 통합본의 장비 재검증은 남아 있다. RViz 관절명·제한 정합과 full/lite 조합은 코드 점검에서 찾은 검증 과제다. 시험 시 기준 SHA, 조건, 로그·영상과 기대값·실제 결과를 함께 확인해야 한다.

---

## 5. 프로젝트 타임라인

| 날짜 | 주요 진행 | 담당과 협업 | 관련 자료 |
| --- | --- | --- | --- |
| 10월 1일 | 역할 분담과 저장소 설정, 제어 펌웨어·모터 확인, 카메라 환경과 예제 확인 | [홍석민](https://github.com/hsmint) 팀 운영, [김혜민](https://github.com/heffeekim94-web) 제어, [최형준](https://github.com/chj1319) 통합, [이홍주](https://github.com/kanichong) 인지 | 회의·투두·개인 일지 |
| 10월 2일 | depth 기반 크기 검증·인지 ROS 노드 구성, 파란 큐브 검출·추적 검토 | [최형준](https://github.com/chj1319), [이홍주](https://github.com/kanichong) | 개인 일지 |
| 10월 3일 | X/Y축 동시 입력과 공통 속도 적용, 토픽·QoS·인지/제어·launch 기본 코드 작성 | [김혜민](https://github.com/heffeekim94-web) 제어, [최형준](https://github.com/chj1319) ROS 통합 | 개인 일지와 해당 날짜의 작성 커밋 |
| 10월 4일 | 제어 PR #2를 develop에 반영 | [김혜민](https://github.com/heffeekim94-web) 작성, [홍석민](https://github.com/hsmint) 병합 | [PR #2](https://github.com/hsmint/ROS-Vision-Tracker/pull/2) |
| 10월 6일 | 제어 정리·URDF 모델 제작 완료, 인지 튜닝, 역할별 패키지·토픽·시리얼 통합, URDF·launch 반영 | #3은 control에 병합. #4·#5·#6·#7·#8은 develop에 병합. [김혜민](https://github.com/heffeekim94-web)이 제작한 URDF를 [홍석민](https://github.com/hsmint)이 저장소·launch에 반영하고 [최형준](https://github.com/chj1319)이 #8을 병합 | 개인 일지와 병합 PR |
| 10월 7일 | cube detector 반영, tracker 구조 통합, lite 제어 전환 | [이홍주](https://github.com/kanichong) detector, [홍석민](https://github.com/hsmint) 통합·제어 변경 | [PR #10](https://github.com/hsmint/ROS-Vision-Tracker/pull/10), 직접 작성 커밋 |
| 10월 8일 | 인지·CSRT·단절 복구 변경 반영. PR #13은 01:36:14 승인 후 01:36:27 병합. 통합 코드 반영 완료, 실기 검증 대기 | [이홍주](https://github.com/kanichong) 작성, [홍석민](https://github.com/hsmint) 승인·병합. PR #11에는 공동 작성 커밋 포함 | [PR #11](https://github.com/hsmint/ROS-Vision-Tracker/pull/11), [PR #13](https://github.com/hsmint/ROS-Vision-Tracker/pull/13), 10월 8일 프로젝트 타임라인 |

10월 8일 프로젝트는 **진행 중**이다. 인지·추적·제어·실행 코드의 develop 통합을 완료했으며, 같은 버전으로 장비 동작과 성능을 다시 검증해야 한다. [김혜민](https://github.com/heffeekim94-web)의 10월 6일 개인 작업은 완료 상태다.

---

## 6. 참고 자료

- [F팀 Xmen Notion 팀 페이지](https://app.notion.com/p/teamsparta/F-_Xmen-3eb2dc3ef514801e8f3edab3247c336d)
