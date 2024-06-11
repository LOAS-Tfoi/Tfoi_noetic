# Tfoi_noetic


<img src="https://github.com/LOAS-Tfoi/Tfoi_noetic/assets/117344855/512ec3d4-b469-47d6-943e-b6222e08e2ad" alt="Tfoi Logo" width="500" height="500">


**버전: 2024.06.11**

이 프로젝트는 2020년 우분투 환경에서 동작하는 Tfoi_noetic의 소프트웨어입니다.

## 최신 업데이트

- **Tfoi_noetic_fw94a**
- **Tfoi_noetic_md94a**

## TFOI 2020 업데이트 목록

### Version : "fw94a"

- 기본 자율 주행 정상 동작 (목표 골 도착, 장애물 인지 및 간접 회피)
- 사선 경로의 골을 지정하였을 경우 헤딩의 어큐런시가 떨어져 목표골을 지나치는 경우가 종종 발생했는데 어큐런시를 높여 문제를 해결함(테스트 4시간 : 목표골 지나친 경우 없음)
- 전방에 장애물이 있을시 멈춤(뒤로가지 않음)
- 측면에 장애물이 있을시 어느정도 여유 공간이 있으면 회피해서 지나감

#### 특이사항

- 사선에 대한 어큐런시를 높였지만, 4시간 사선 경로 테스트시 20바퀴중 한번마다 특정경로에서 패스를 정하기 위해 헤딩을 좌우로 5번 정도 움직임(2차 어큐런시 보완코드로 해결할것임)

### 자율 주행 알고리즘 외 추가 기능

- 웹 데이터 퍼블리셔 패키지 추가(배터리 전압, 용량, 전류, 속도, 이동거리등 웹[관제] 에서 관리 하는 데이터 퍼블리쉬)
- 서비스 콜 서버, 클라이언트 패키지 업데이트(웹 연계 서비스콜 스위칭 동작 기능 구현)

**총 패키지: 48개 → 51개 (업데이트)**

## 사용 방법

1. `catkin_ws` 디렉토리에 존재하는 모든 폴더 및 파일을 삭제합니다.
2. `catkin_ws에 /src` 디렉터리를 새로 만들고 아래 명령을 실행하여 코드를 다운로드합니다.

```bash
git clone https://github.com/LOAS-Tfoi/Tfoi_noetic.git
```

### 웹 데이터 퍼블리셔 Topic list
```bash
/tfoi_voltage          Tfoi  배터리 전압[V]          Type : std_msgs::Float32
/tfoi_current          Tfoi  배터리 전류[A]          Type : std_msgs::Float32
/tfoi_capacity         Tfoi  배터리 용량[C]          Type : std_msgs::Float32
/tfoi_battery          Tfoi  배터리 상태[%]          Type : std_msgs::UInt8
/tfoi_temperature      Tfoi  배터리 온도             Type : std_msgs::Float32
/tfoi_over_voltage     Tfoi  배터리 과전압 플래그      Type : std_msgs::Bool
/tfoi_charge_flag      Tfoi  배터리 충전 플래그        Type : std_msgs::Bool
/tfoi_distance         Tfoi  Tfoi 이동 거리          Type : std_msgs::Float32  (테스트 및 검증 필요)


