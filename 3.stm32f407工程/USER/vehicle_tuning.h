#ifndef VEHICLE_TUNING_H
#define VEHICLE_TUNING_H
/* �����ֳ�������ڡ���λд�����ƺ�ע���У��޸ĺ�������ء�
 * ��ǰֻʵ�ֵ�һ�׶Σ���ͣ��1ǰ�������Ƶ�ɨ��㣻���Զ�����ڶ��׶Ρ�
 * ��ͼ���û�ͼ�������ֹ��㣬��������δ���״���ʵ������ȫһ�¡� */
#define VEHICLE_STAGE1_ENABLE 1U
#define VEHICLE_MAP_VERSION 2U
#define FIELD_SIZE_MM 2400.0f
#define FIELD_GRID_SIZE_MM 480.0f
/* �û��������꣺+X����ͣ��2ָ����ͣ��1��+Y����ͣ��2ָ��ּӹ�����
 * ��ͷ��+Y/ԭ������������-X������Ŀ���겻ð���״��豸���ꡣ */
#define FIELD_START1_X_MM 2250.0f
#define FIELD_START1_Y_MM 150.0f
#define FIELD_START2_X_MM 150.0f
#define FIELD_START2_Y_MM 150.0f
/* �û�ȷ�ϣ���ɨ��ģ������β��������������100mm����һ�׶�
 * ��ǰ��15cm������105cm������ת��ɨ���Ϊ�����е���+Yǰ��150mm��
 * �޸�ǰ�������FORWARD_CM���޸����ƾ���ɵ���SCAN_X_MM��
 * Ĭ��ɨ��ͣ����(1200,300)mm��ԭ�����е�(1200,150)ֻ�����βο��� */
#define VEHICLE_STAGE1_FORWARD_CM 15.0f
#define FIELD_SCAN_X_MM ((FIELD_START1_X_MM+FIELD_START2_X_MM)*0.5f)
#define FIELD_SCAN_Y_MM (FIELD_START1_Y_MM+VEHICLE_STAGE1_FORWARD_CM*10.0f)
/* ������λĬ��ȡ�����ģ������ֳ��ڴ˶����޸�ͣ�����ꡣ */
#define FIELD_RAW_X_MM 2160.0f
#define FIELD_RAW_Y_MM 1200.0f
#define FIELD_COARSE_X_MM 240.0f
#define FIELD_COARSE_Y_MM 1200.0f
#define FIELD_STORAGE_X_MM 1200.0f
#define FIELD_STORAGE_Y_MM 2160.0f
/* ��һ�׶α��ֳ�ͷ��+Y��ǰ��=+Y������=-X��ֻ֧��PA1��ͣ��1�� */
#define VEHICLE_STAGE1_ZONE 1U
#define VEHICLE_STAGE1_ENTRY_SPEED_CM_S 5.0f
#define VEHICLE_STAGE1_SPEED_CM_S 10.0f
#define VEHICLE_STAGE1_MOVE_TIMEOUT_MS 60000U
#define VEHICLE_STAGE1_SCAN_TIMEOUT_MS 8000U
#define VEHICLE_STAGE1_SCAN_FRESH_MS 500U
/* �����û����ܵ�������������������²���������������
 * ɨ�봰������β���ݰ����ĺ󷽰복�����㣻���/ͻ�������ֳ�У׼��
 * ���������Ҿ��е�X������ࡣ�������û�ȷ��ΪY=0��
 * Ĭ��ͣ��Y=300�����崰��Y=300-141.5=158.5mm�����ܳ�ʵ�⡣ */
#define VEHICLE_BODY_LENGTH_MM 283.0f
#define VEHICLE_BODY_WIDTH_MM 254.0f
#define FIELD_SCANNER_REAR_OFFSET_MM (VEHICLE_BODY_LENGTH_MM*0.5f)
#define FIELD_QR_PLANE_Y_MM 0.0f
#define VEHICLE_SCAN_MIN_STANDOFF_MM 100.0f
#define VEHICLE_STAGE1_SCAN_AFTER_STOP 1U
/* ��ǰ���μ�����300mm���ΰ��磻��ʵ������/չ��ɨ�������ա� */
#define FIELD_BODY_HALF_MM 150.0f
#if VEHICLE_STAGE1_ENABLE > 1U || VEHICLE_STAGE1_ZONE != 1U
#error Stage1_currently_supports_start_zone1_only
#endif
#if VEHICLE_STAGE1_SCAN_AFTER_STOP > 1U
#error Invalid_stage1_scan_option
#endif
#endif
