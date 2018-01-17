#ifndef __JOINT_H__
#define __JOINT_H__
#include "mrapi.h"
#include "module.h"

#define CMDMAP_LEN            160    //

/// proctocol definition
//ϵͳ״̬���
#define SYS_ID                0x01    //������ID
#define SYS_MODEL_TYPE        0x02    //�������ͺ�
#define SYS_FW_VERSION        0x03    //�̼��汾
#define SYS_ERROR             0x04    //�������
#define SYS_VOLTAGE           0x05    //ϵͳ��ѹ
#define SYS_TEMP              0x06    //ϵͳ�¶�
#define SYS_REDU_RATIO        0x07    //ģ����ٱ�
//#define SYS_BAUDRATE_232      0x08    //232�˿ڲ�����
#define SYS_BAUDRATE_CAN      0x09    //CAN���߲�����
#define SYS_ENABLE_DRIVER     0x0a    //������ʹ�ܱ�־
#define SYS_ENABLE_ON_POWER   0x0b    //�ϵ�ʹ����������־
#define SYS_SAVE_TO_FLASH     0x0c    //�������ݵ�Flash��־
//#define SYS_DEMA_ABSPOS       0x0d    //�Զ��궨����λ�ñ�־
#define SYS_SET_ZERO_POS      0x0e    //����ǰλ������Ϊ����־
#define SYS_CLEAR_ERROR       0x0f    //��������־

#define SYS_CURRENT_L         0x10    //��ǰ������16λ��mA��
#define SYS_CURRENT_H         0x11    //��ǰ������16λ��mA��
#define SYS_SPEED_L           0x12    //��ǰ�ٶȵ�16λ��units/s��
#define SYS_SPEED_H           0x13    //��ǰ�ٶȸ�16λ��units/s��
#define SYS_POSITION_L        0x14    //��ǰλ�õ�16λ��units��
#define SYS_POSITION_H        0x15    //��ǰλ�ø�16λ��units��
#define SYS_POTEN_VALUE       0x16    //���ֵ�λ��ֵ
#define SYS_ZERO_POS_OFFSET_L 0x17    //���λ��ƫ������16λ��units��
#define SYS_ZERO_POS_OFFSET_H 0x18    //���λ��ƫ������16λ��units��

//��������Ϣ
#define MOT_RES               0x20    //�������
#define MOT_INDUC             0x21    //������
#define MOT_RATED_VOL         0x22    //������ѹ
#define MOT_RATED_CUR         0x23    //��������
//#define MOT_ENC_LINES         0x24    //��������
//#define MOT_HALL_VALUE        0x25    //��ǰ����״̬
#define MOT_ST_DAT            0x26    //���Ա�������Ȧ����
#define MOT_MT_DAT            0x27    //���Ա�������Ȧ����
#define MOT_ENC_STA           0x28    //���Ա�����״̬�Ĵ���
#define BAT_VOLT              0x29    //��������ص�ѹ *10mV
#define ACC_X                 0x2A    //���ٶȼ�x�� *1000mg
#define ACC_Y                 0x2B    //���ٶȼ�y�� *1000mg
#define ACC_Z                 0x2C    //���ٶȼ�z�� *1000mg

//����Ŀ��ֵ
#define TAG_WORK_MODE         0x30    //����ģʽ��0-������1-����ģʽ��2-�ٶ�ģʽ��3-λ��ģʽ
#define TAG_OPEN_PWM          0x31    //����ģʽ��ռ�ձȣ�0~100��
#define TAG_CURRENT_L         0x32    //Ŀ�������16λ��mA��
#define TAG_CURRENT_H         0x33    //Ŀ�������16λ��mA��
#define TAG_SPEED_L           0x34    //Ŀ���ٶȵ�16λ��units/s��
#define TAG_SPEED_H           0x35    //Ŀ���ٶȸ�16λ��units/s��
#define TAG_POSITION_L        0x36    //Ŀ��λ�õ�16λ��units��
#define TAG_POSITION_H        0x37    //Ŀ��λ�ø�16λ��units��

//��������ֵ
#define LIT_MAX_CURRENT       0x40    //��������mA��
#define LIT_MAX_SPEED         0x41    //����ٶȣ�rpm��
#define LIT_MAX_ACC           0x42    //�����ٶȣ�rpm/s��
#define LIT_MIN_POSITION_L    0x43    //��Сλ�õ�16λ��units��
#define LIT_MIN_POSITION_H    0x44    //��Сλ�ø�16λ��units��
#define LIT_MAX_POSITION_L    0x45    //���λ�õ�16λ��units��
#define LIT_MAX_POSITION_H    0x46    //���λ�ø�16λ��units��

//���ջ������
#define SEV_PARAME_LOCKED     0x50    //���ջ�����������־, 0-������(�Զ��л�), 1-���٣�S��,2-���٣�M��,3-���٣�L��

#define S_CURRENT_P           0x51    //������P����
#define S_CURRENT_I           0x52    //������I����
#define S_CURRENT_D           0x53    //������D����
#define S_SPEED_P             0x54    //�ٶȻ�P����
#define S_SPEED_I             0x55    //�ٶȻ�I����
#define S_SPEED_D             0x56    //�ٶȻ�D����
#define S_SPEED_DS            0x57    //�ٶ�P����
#define S_POSITION_P          0x58    //λ�û�P����
#define S_POSITION_I          0x59    //λ�û�I����
#define S_POSITION_D          0x5A    //λ�û�D����
#define S_POSITION_DS         0x5B    //λ��P����
#define S_CURRENT_FD          0x5C    //����ǰ��
#define M_CURRENT_FD          0x5D    //����ǰ��
#define L_CURRENT_FD          0x5E    //����ǰ��

#define M_CURRENT_P           0x61    //������P����
#define M_CURRENT_I           0x62    //������I����
#define M_CURRENT_D           0x63    //������D����
#define M_SPEED_P             0x64    //�ٶȻ�P����
#define M_SPEED_I             0x65    //�ٶȻ�I����
#define M_SPEED_D             0x66    //�ٶȻ�D����
#define M_SPEED_DS            0x67    //�ٶ�P����
#define M_POSITION_P          0x68    //λ�û�P����
#define M_POSITION_I          0x69    //λ�û�I����
#define M_POSITION_D          0x6A    //λ�û�D����
#define M_POSITION_DS         0x6B    //λ��P����

#define L_CURRENT_P           0x71    //������P����
#define L_CURRENT_I           0x72    //������I����
#define L_CURRENT_D           0x73    //������D����
#define L_SPEED_P             0x74    //�ٶȻ�P����
#define L_SPEED_I             0x75    //�ٶȻ�I����
#define L_SPEED_D             0x76    //�ٶȻ�D����
#define L_SPEED_DS            0x77    //�ٶ�P����
#define L_POSITION_P          0x78    //λ�û�P����
#define L_POSITION_I          0x79    //λ�û�I����
#define L_POSITION_D          0x7A    //λ�û�D����
#define L_POSITION_DS         0x7B    //λ��P����

//ɲ����������
#define BRAKE_RELEASE_CMD     0x80    //ɲ���ͷ����0-�����ƶ���1-�ͷ�ɲ��
#define BRAKE_STATE           0x81    //ɲ��״̬��0-�����ƶ���1-�ͷ�ɲ��

//ʾ����ģ����������ַ����
#define SCP_MASK              0x90    //��¼�����־MASK
#define SCP_REC_TIM           0x91    //��¼ʱ��������10kHZ�ķ�Ƶֵ��

#define SCP_TAGCUR_L          0x92    //Ŀ��������ݼ�
#define SCP_TAGCUR_H          0x93    //Ŀ��������ݼ�
#define SCP_MEACUR_L          0x94    //ʵ�ʵ������ݼ�
#define SCP_MEACUR_H          0x95    //ʵ�ʵ������ݼ�
#define SCP_TAGSPD_L          0x96    //Ŀ���ٶ����ݼ�
#define SCP_TAGSPD_H          0x97    //Ŀ���ٶ����ݼ�
#define SCP_MEASPD_L          0x98    //ʵ���ٶ����ݼ�
#define SCP_MEASPD_H          0x99    //ʵ���ٶ����ݼ�
#define SCP_TAGPOS_L          0x9A    //Ŀ��λ�����ݼ�
#define SCP_TAGPOS_H          0x9B    //Ŀ��λ�����ݼ�
#define SCP_MEAPOS_L          0x9C    //ʵ��λ�����ݼ�
#define SCP_MEAPOS_H          0x9D    //ʵ��λ�����ݼ�

//ʾ������¼����MASK����
#define MASK_TAGCUR         0x0001    //��¼Ŀ�����MASK
#define MASK_MEACUR         0x0002    //��¼ʵ�ʵ���MASK
#define MASK_TAGSPD         0x0004    //��¼Ŀ���ٶ�MASK
#define MASK_MEASPD         0x0008    //��¼ʵ���ٶ�MASK
#define MASK_TAGPOS         0x0010    //��¼Ŀ��λ��MASK
#define MASK_MEAPOS         0x0020    //��¼ʵ��λ��MASK


typedef uint8_t rec_t[8];

typedef struct td_joint
{
	Module* basicModule;
	uint16_t* jointId;
	uint16_t* jointType;
	uint8_t isOnline;

	rec_t txQue[MAX_BUFS];
	uint16_t txQueFront;
	uint16_t txQueRear;
	jQueShortHandler_t jointBufUnderflowHandler;
}Joint;

///For advanced users
int32_t jointGet(uint8_t index, uint8_t datLen, Joint* pJoint, void* data, int32_t timeout, jCallback_t callBack);
int32_t jointSet(uint8_t index, uint8_t datLen, Joint* pJoint, void* data, int32_t timeout, jCallback_t callBack);
#endif

