# Tfoi_noetic
2020 우분투 Tfoi_noetic

version upload 

2024.06.05 
  - Tfoi_noetic_90d

2024.06 ~ 개발 업데이트 사항
- 알고리즘 업데이트
  1) 목표지점 한번 놓치면 런치를 다시 실행할 때 까지 다음 목표점을 인지 못하는 버그 -> 해결
  2) 목표점이 안맞았을 때 Yaw 각도 피드백 제어
  3) 장애물 회피후 목표 지점 도착 확률 up(개발중)
  4) "cantransform tf2_ros ... " 에러 로그가 지속적으로 발생하는 문제 해결중
  5) local_path_plan 데이터를 가져와 패스에 맞는 좌표변환 후 이전 키네틱 코드의 곡률 제어 함수 적용 코드 개발중 
