// ======================================= TFOI 2020 Update List  =======================================


// *Version : "fw97b"

//            - 95a, 97a 버전에 비해 끼임 현상 완화
//            - 장애물 감지 후 멈춤 업그레이드
//            - 헤딩 좌우 모션 버그 완화  
  
// 자율 주행 알고리즘 외 추가 기능 
//  - 웹 데이터 퍼블리셔 패키지 추가(배터리 전압, 용량, 전류, 속도, 이동거리등 웹[관제] 에서 관리 하는 데이터 퍼블리쉬)
//  - 서비스 콜 서버, 클라이언트 패키지 업데이트(웹 연계 서비스콜 스위칭 동작 기능 구현)

// 총 패키지 48개 -> 51개 (update)

// teb_local_planner.cpp  debug

/*

#if 1
  #define ROS_DEBUG_TEST_01 0 // teb_path data 출력
  #define ROS_DEBUG_TEST_02 0 // teb_path.pose data 확인
  #define ROS_DEBUG_TEST_03 0 // teb_path.pose -> base_link 좌표변환 데이터 확인 
  #define ROS_DEBUG_TEST_04 0 // teb_path.pose의 각 벡터간 거리 체크
  #define ROS_DEBUG_TEST_05 0 // 로봇의 현재 pose 확인(로봇 기준)
  #define ROS_DEBUG_TEST_06 1 // 목표점 거리 허용 오차 범위내 도착 로그
  #define ROS_DEBUG_TEST_07 1 // 목표점 헤딩 허용 오차 범위내 도착 로그 
  #define ROS_DEBUG_TEST_08 0 // 로봇이 목표방향으로 돌려야 하는 헤딩 각도(rad)
  #define ROS_DEBUG_TEST_09 0 // 로봇위치 기준 목표점과의 거리(distance)
  #define ROS_DEBUG_TEST_10 0 // teb_path 변환 roll, pitch, yaw 쿼터니언 
  #define ROS_DEBUG_TEST_11 1 // interval time 체크 
  #define ROS_DEBUG_ERROR_01 0 // teb_path.pose TF 변환 에러
  #define ROS_DEBUG_ERROR_02 1 // Tolerance GOAL Distance OK,Tolerance GOAL Yaw NG
  #define ROS_DEBUG_ERROR_03 1 // Tolerance GOAL Distance NG
#else
  #define ROS_DEBUG_TEST_01 1 // teb_path data 출력
  #define ROS_DEBUG_TEST_02 1 // teb_path.pose data 확인
  #define ROS_DEBUG_TEST_03 1 // teb_path.pose -> base_link 좌표변환 데이터 확인 
  #define ROS_DEBUG_TEST_04 1 // teb_path.pose의 각 벡터간 거리 체크
  #define ROS_DEBUG_TEST_05 1 // 로봇의 현재 pose 확인(로봇 기준)
  #define ROS_DEBUG_TEST_06 1 // 목표점 거리 허용 오차 범위내 도착 로그
  #define ROS_DEBUG_TEST_07 1 // 목표점 헤딩 허용 오차 범위내 도착 로그 
  #define ROS_DEBUG_TEST_08 1 // 로봇이 목표방향으로 돌려야 하는 헤딩 각도(rad)
  #define ROS_DEBUG_TEST_09 1 // 로봇위치 기준 목표점과의 거리(distance)
  #define ROS_DEBUG_TEST_10 1 // teb_path 변환 roll, pitch, yaw 쿼터니언 
  #define ROS_DEBUG_TEST_11 1 // interval time 체크 
  #define ROS_DEBUG_ERROR_01 1 // teb_path.pose TF 변환 에러
  #define ROS_DEBUG_ERROR_02 1 // Tolerance GOAL Distance OK,Tolerance GOAL Yaw NG
  #define ROS_DEBUG_ERROR_03 1 // Tolerance GOAL Distance NG  
#endif


*/

// 디버깅 코드 작성 예시 

/*
// 조건이 TRUE일때만 컴파일 됨 
#if ROS_DEBUG_TEST_01
ROS_ERROR(" Tfoi was goal reached. (x , y)");
#endif

// 조건이 TRUE일때만 컴파일 됨
#if ROS_DEBUG_TEST_02
ROS_ERROR(" Tfoi was goal reached. (Yaw) ");
#endif

// 조건이 TRUE일때만 컴파일 됨 

#if ROS_DEBUG_TEST_03
ROS_ERROR("Detected collision ahead!");
#endif

*/