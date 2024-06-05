// ======================================= TFOI 2020 Update List  =======================================


// *Version : "90d"
//  - 목표점과 근접했을 때 헤딩이 안맞을 시 헤딩 피드백 제어
//  - 장애물 전방, 측면 인지후 목표점에 어느 정도 도달함 + 피드백 제어

// 결과 : 웹단에서 네비게이션 동작이 되지 않는 부분은 디버깅 테스트로 해결함 하지만 알고리즘 부분의 코드를 더욱 수정 및 보완이 필요함


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