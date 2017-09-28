

typedef enum
{
    WAIT         ,
    FRAME_BEGIN  ,
    ACCEL_X      ,
    ACCEL_Y      ,
    ACCEL_Z      ,
    TEMP         ,
    GYRO_X       ,
    GYRO_Y       ,
    GYRO_Z       ,
    MAG_X        ,
    MAG_Y        ,
    MAG_Z        ,
    CRC_CHECK    ,
    FRAME_END    ,
    FRAME_SHOW   ,
    FRAME_FILTER ,
    CRC_ERR      ,
} DECODE_STATUS;

typedef struct
{
    unsigned char accel_xh;
    unsigned char accel_xl;
    unsigned char accel_yh;
    unsigned char accel_yl;
    unsigned char accel_zh;
    unsigned char accel_zl;

    short accel_X;
    short accel_Y;
    short accel_Z;
}accelermeter_data;

typedef struct
{
    unsigned char gyro_xh;
    unsigned char gyro_xl;
    unsigned char gyro_yh;
    unsigned char gyro_yl;
    unsigned char gyro_zh;
    unsigned char gyro_zl;

    short gyro_X;
    short gyro_Y;
    short gyro_Z;
}gyroscope_data;

typedef struct
{
    unsigned char mag_xh;
    unsigned char mag_xl;
    unsigned char mag_yh;
    unsigned char mag_yl;
    unsigned char mag_zh;
    unsigned char mag_zl;


    short mag_X;
    short mag_Y;
    short mag_Z;
}magnetic_data;

typedef struct
{
    unsigned char pressh;
    unsigned char pressl;

    short Pressure;
}pressure_data;


typedef struct
{
    unsigned char temph;
    unsigned char templ;

    short Tempreture;

}tempreture_data;


typedef struct
{
    accelermeter_data accel_raw;
    gyroscope_data    gyro_raw;
    magnetic_data     mag_raw;
    pressure_data     press_raw;
    tempreture_data   temp_raw;

}MPUSensor_RAW;







