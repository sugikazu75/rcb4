import cstruct


ServoStruct = cstruct.parse("""
struct ServoStruct
{
  uint8_t flag; // 1: available
  uint8_t mode; //  servo EEPROM address 14,15
  uint8_t rotation; // 1:rotation servo, 0:hinge servo
  uint8_t port; // 1:J1:huart2, 4:J2:huart7,
                // 6:J3:huart5, 2:J4:huart4, 3:J5:huart3, 5:J6:huart1
  //
  uint16_t ref_angle;
  uint16_t current_angle;
  int16_t trim;
  uint16_t start_ref_angle;
  uint16_t cmd_ref_angle;
  uint16_t prev_angle;
  //
  uint8_t params[64];
  //
  uint8_t frame;
  uint8_t stretch;
  uint8_t speed;
  uint8_t current_limit;
  uint8_t temperature_limit;
  // eeprom
  uint8_t eeprom_update;
  uint8_t timeout_count;
  uint8_t rxerror_count;
  uint32_t count_frame;
  uint32_t prev_frame;

  uint8_t init_state; //0:no connection 1:initialization_state 2:initialized
  uint8_t init_count;
  int16_t error_angle;

  float p_gain;    // for rotation servo
  float i_gain;    // for rotation servo
  float d_gain;    // for rotation servo
  float e_sum;    // for rotation servo
  uint8_t feedback;  // for rotation servo 1: on 0:off
  uint8_t move_state; // 0:stop or interpolated, 1:moving
  uint8_t current;    // servo current CW:0 - 63, CCW:64 -127
  uint8_t temperature;  // servo temperature: 1 - 127
};
""")
ServoStruct.__name__ = 'servo_vector'


SensorbaseStruct = cstruct.parse("""
#define SC_NONE 0x40
#define SC_SW_ADC 0x41
#define SC_PS  0x42
#define SC_IMU 0x44
#define SC_MAGENC 0x48
#define SC_ALL (SC_SW_ADC | SC_PS | SC_IMU | SC_MAGENC)
#define SC_SW_ADC_R (SC_SW_ADC & 0x0F)
#define SC_PS_R (SC_PS & 0x0F)
#define SC_IMU_R (SC_IMU & 0x0F)
#define SC_MAGENC_R (SC_MAGENC & 0x0F)
#define ADC_CHANNEL_NUM 5
#define FORCE_CHANNEL_NUM 4
#define PS_CHANNEL_NUM 4
#define GYRO_CHANNEL_NUM 3
#define ACC_CHANNEL_NUM 3
#define MAGENC_CHANNEL_NUM 1
#define ADC_LEN ADC_CHANNEL_NUM * 2
#define PS_LEN PS_CHANNEL_NUM * 3
#define GYRO_LEN GYRO_CHANNEL_NUM * 3
#define ACC_LEN ACC_CHANNEL_NUM * 3
#define IMU_LEN GYRO_LEN + ACC_LEN
#define MAGENC_LEN MAGENC_CHANNEL_NUM * 2
#define MAX_BUFFER_LENGTH 2 + ADC_LEN + PS_LEN + IMU_LEN + MAGENC_LEN

struct SensorbaseStruct
{
    uint8_t port;
    uint8_t id;
    uint8_t mode;
    uint8_t sw;
    //uint16_t adc[ADC_CHANNEL_NUM]; TODO:for KJS voltage measurement
    uint16_t adc[FORCE_CHANNEL_NUM];//adc for force sensor
    uint16_t ps[PS_CHANNEL_NUM];
    int16_t gyro[GYRO_CHANNEL_NUM];
    int16_t acc[ACC_CHANNEL_NUM];
    uint16_t rxerror_count;
    uint16_t timeout_count;
    uint8_t debug_rxbuff[2];
    uint32_t count_frame;//us
    uint32_t prev_frame;//us
    uint16_t adc_calib[(FORCE_CHANNEL_NUM + 1) * 5];//for FSR calibration
    float force[FORCE_CHANNEL_NUM];//caliculated from adc val
    uint16_t temperature;
    uint16_t magenc;

    uint16_t adc_offset[FORCE_CHANNEL_NUM];//initial adc value
    float adc_calib_hsf[4 * (FORCE_CHANNEL_NUM + 1)];
    // for HSFPAR003A force calibration
    // [bFz,aFz0~aFz3],[bMx,aMx0~aMx3],[bMy,aMy0~aMy3],
    // [Fz scale[N -> g], Mx scale[Nm -> Nmm], My scale[Nm -> Nmm],
    // calib_paramA, calib_paramB]
    float wrench[3];//[Fz [g],Mx [Nmm],My[Nmm]]
    float wrench_offset[3];//[Fz [g],Mx [Nmm],My[Nmm]]
    float wrench_scale[3];//[Fz [g],Mx [Nmm],My[Nmm]]
    uint8_t wrench_init_count;
    uint8_t wrench_init_done;
    uint8_t sensor_updated;
    uint8_t is_contact;
    uint8_t board_revision;
    uint8_t move_state;
    uint8_t dummy[2];
};
""")
SensorbaseStruct.__name__ = 'Sensor_vector'

ImuData = cstruct.parse("""
struct ImuData_t
{
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];
}
""")
ImuData.__name__ = 'imu_data_'

c_vector = {
    'servo_vector': 36,
    'Sensor_vector': 0,
    'imu_data_': 0,
}
