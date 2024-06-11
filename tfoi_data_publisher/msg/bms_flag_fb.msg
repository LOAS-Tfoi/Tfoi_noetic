uint8 bms_flag_fb_soc                 # 배터리 상태 소숫점 표시(최대,최저치가 몇인지 테스트 필요)
bool bms_flag_fb_single_ov            # 1개의 배터리 셀 과전압 상태  
bool bms_flag_fb_single_uv            # 1개의 배터리 셀 저전압 상태
bool bms_flag_fb_ov                   # Over Voltage : 배터리 전압 허용 범위 초과
bool bms_flag_fb_uv                   # Under Voltage : 만터리 전압 허용 범위 미만
bool bms_flag_fb_charge_ot            # 배터리 충전중 과온 상태(충전중 배터리 과열)
bool bms_flag_fb_charge_ut            # 배터리 충천중 저온 상태 
bool bms_flag_fb_discharge_ot         # 배터리 방전시 과온 상태
bool bms_flag_fb_discharge_ut         # 배터리 방전시 저온 상태
bool bms_flag_fb_charge_oc            # 배터리 충전시 과전류 상태
bool bms_flag_fb_discharge_oc         # 배터리 방전시 과전류 상태
bool bms_flag_fb_short                # 배터리 회로 쇼트
bool bms_flag_fb_ic_error             # 배터리 관리 시스템 집적회로(IC) 오류
bool bms_flag_fb_lock_mos             # MOSFET lock
bool bms_flag_fb_charge_flag          # 충전 상태 플래그
float32 bms_flag_fb_hight_temperature # 배터리의 높은 온도 상태
float32 bms_flag_fb_low_temperature   # 배터리의 낮은 온도 상태
