#include "stdlib.h"
#include "iostream"
#include "stdio.h"
#include "sstream"

#include "serial_port.h"
#include "ahrs_serial.h"
#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

using namespace std;
using namespace ros;

//#define SENSOR_SHOW
//#define JUMP_SHOW
//#define SERIAL_DATA_SHOW
//#define SENSOR_ZERO_CRRECT
#define SENSOR_FILTER
//#define CRC_SHOW
#define FRAME_COUNTER



DECODE_STATUS     step;
MPUSensor_RAW     mpu9250;
int fd;

ros::Publisher imu_pub;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;



const float accel_range    = 4.000000;
const float gyro_range     = 250.000000;
const float magn_range     = 1;
const float temp_range     = 1;
const float pressure_range = 1;
const float g              = 9.80665;
const float pi             = 3.1415926;

char raw_data[20];
int crc_res;
int crc_gen;
unsigned char crc_l,crc_h;
int dat_count;
long frame_num;
long error_frame_num;
unsigned char buff[1];
unsigned char dat;

/*******************************CRC16 Check code****************************/
int calcByte(int crc, char b)
{
    int i;
    crc = crc ^ (int)b << 8;

    for ( i = 0; i < 8; i++)
    {
        if ((crc & 0x8000) == 0x8000)
            crc = crc << 1 ^ 0x1021;
        else
            crc = crc << 1;
    }

    return crc & 0xffff;
}

int CRC16(char *pBuffer, int length)

{
    int wCRC16=0;
    int i;
    if (( pBuffer==0 )||( length==0 ))
    {
        return 0;
    }
    for ( i = 0; i < length; i++)
    {
        wCRC16 = calcByte(wCRC16, pBuffer[i]);
    }
    return wCRC16;
}
/******************************************************************************/


void decode_loop()
{


    UART_Recv(fd,buff,1);
    dat = *buff;
#ifdef SERIAL_DATA_SHOW
    printf("dat: %x\n", dat);
#endif
    switch(step)
    {
    case WAIT:
        step = WAIT;
        break;
    case FRAME_BEGIN:
        if(dat == 0xaa || dat == 0xbb)
        {
            dat_count++;
        }
        else
        {
            dat_count = 0;
        }
        if(dat_count == 2)
        {
            step = ACCEL_X;
            dat_count = 0;
        }

        break;
    case ACCEL_X:
#ifdef JUMP_SHOW
        cout << "go to ACCEL_X" << endl;
#endif
        if(dat_count == 0)
        {
            mpu9250.accel_raw.accel_xh = dat; raw_data[0] = dat;
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.accel_raw.accel_xl = dat;  raw_data[1] = dat;
            mpu9250.accel_raw.accel_X =
                    (((short)mpu9250.accel_raw.accel_xh) << 8)
                    | mpu9250.accel_raw.accel_xl;
            dat_count = 0;
            imu_msg.linear_acceleration.x =
                    g*accel_range*(float)mpu9250.accel_raw.accel_X/(float)32768;
            step = ACCEL_Y;
        }
        break;
    case ACCEL_Y:
#ifdef JUMP_SHOW
        cout << "go to ACCEL_Y" << endl;
#endif
        if(dat_count == 0)
        {
            mpu9250.accel_raw.accel_yh = dat; raw_data[2] = dat;
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.accel_raw.accel_yl = dat; raw_data[3] = dat;
            mpu9250.accel_raw.accel_Y =
                    (((short)mpu9250.accel_raw.accel_yh) << 8)
                    | mpu9250.accel_raw.accel_yl;
            dat_count = 0;
            imu_msg.linear_acceleration.y =
                    g*accel_range*(float)mpu9250.accel_raw.accel_Y/(float)32768;
            step = ACCEL_Z;
        }
        break;

    case ACCEL_Z:
#ifdef JUMP_SHOW
        cout << "go to ACCEL_Z" << endl;
#endif
        if(dat_count == 0)
        {
            mpu9250.accel_raw.accel_zh = dat; raw_data[4] = dat;
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.accel_raw.accel_zl = dat; raw_data[5] = dat;
            mpu9250.accel_raw.accel_Z =
                    (((short)mpu9250.accel_raw.accel_zh) << 8)
                    | mpu9250.accel_raw.accel_zl;
            dat_count = 0;
            imu_msg.linear_acceleration.z =
                    g*accel_range*(float)mpu9250.accel_raw.accel_Z/(float)32768;
            step = TEMP;
        }
        break;
    case TEMP:
#ifdef JUMP_SHOW
        cout << "go to TEMP" << endl;
#endif

        if(dat_count == 0)
        {
            mpu9250.temp_raw.temph = dat; raw_data[6] = dat;
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.temp_raw.templ = dat; raw_data[7] = dat;
            mpu9250.temp_raw.Tempreture =
                    (((short)mpu9250.temp_raw.temph) << 8)
                    | mpu9250.temp_raw.templ;
            dat_count = 0;
            step = GYRO_X;
        }
        break;

    case GYRO_X:
#ifdef JUMP_SHOW
        cout << "go to GYRO_X" << endl;
#endif
        if(dat_count == 0)
        {
            mpu9250.gyro_raw.gyro_xh = dat; raw_data[8] = dat;
            //printf("dat: %x-", dat);
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.gyro_raw.gyro_xl = dat; raw_data[9] = dat;
            //printf("%x ", dat);
            mpu9250.gyro_raw.gyro_X =
                    (((short)mpu9250.gyro_raw.gyro_xh) << 8)
                    | mpu9250.gyro_raw.gyro_xl;
            imu_msg.angular_velocity.x =
                    (pi*gyro_range*(float)mpu9250.gyro_raw.gyro_X/(float)32768)/(float)180;
            dat_count = 0;
            step = GYRO_Y;
        }
        break;

    case GYRO_Y:
#ifdef JUMP_SHOW
        cout << "go to GYRO_Y" << endl;
#endif

        if(dat_count == 0)
        {
            mpu9250.gyro_raw.gyro_yh = dat; raw_data[10] = dat;
            //printf("%x-", dat);
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.gyro_raw.gyro_yl = dat; raw_data[11] = dat;
            //printf("%x ", dat);
            mpu9250.gyro_raw.gyro_Y =
                    (((short)mpu9250.gyro_raw.gyro_yh) << 8)
                    | mpu9250.gyro_raw.gyro_yl;
            imu_msg.angular_velocity.y =
                    (pi*gyro_range * (float)mpu9250.gyro_raw.gyro_Y/(float)32768)/(float)180;
            dat_count = 0;
            step = GYRO_Z;
        }
        break;

    case GYRO_Z:
#ifdef JUMP_SHOW
        cout << "go to GYRO_Z" << endl;
#endif

        if(dat_count == 0)
        {
            mpu9250.gyro_raw.gyro_zh = dat; raw_data[12] = dat;
            //printf("%x-", dat);
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.gyro_raw.gyro_zl = dat; raw_data[13] = dat;
            //printf("%x\n", dat);
            mpu9250.gyro_raw.gyro_Z =
                    (((short)mpu9250.gyro_raw.gyro_zh) << 8)
                    | mpu9250.gyro_raw.gyro_zl;
            imu_msg.angular_velocity.z =
                    (pi*gyro_range * (float)mpu9250.gyro_raw.gyro_Z/(float)32768)/(float)180;
            dat_count = 0;
            step = MAG_X;
        }
        break;


    case MAG_X:
#ifdef JUMP_SHOW
        cout << "go to MAG_X" << endl;
#endif

        if(dat_count == 0)
        {
            mpu9250.mag_raw.mag_xh = dat; raw_data[14] = dat;
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.mag_raw.mag_xl = dat; raw_data[15] = dat;
            mpu9250.mag_raw.mag_X =
                    (((short)mpu9250.mag_raw.mag_xh) << 8)
                    | mpu9250.mag_raw.mag_xl;
            dat_count = 0;
            mag_msg.magnetic_field.x = mpu9250.mag_raw.mag_X;
            step = MAG_Y;
        }
        break;

    case MAG_Y:
#ifdef JUMP_SHOW
        cout << "go to MAG_Y" << endl;
#endif

        if(dat_count == 0)
        {
            mpu9250.mag_raw.mag_yh = dat; raw_data[16] = dat;
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.mag_raw.mag_yl = dat; raw_data[17] = dat;
            mpu9250.mag_raw.mag_Y =
                    (((short)mpu9250.mag_raw.mag_yh) << 8)
                    | mpu9250.mag_raw.mag_yl;
            dat_count = 0;
            mag_msg.magnetic_field.y = mpu9250.mag_raw.mag_Y;
            step = MAG_Z;
        }
        break;

    case MAG_Z:
#ifdef JUMP_SHOW
        cout << "go to MAG_Z" << endl;
#endif

        if(dat_count == 0)
        {
            mpu9250.mag_raw.mag_zh = dat; raw_data[18] = dat;
            dat_count++;
        }
        else if(dat_count == 1)
        {
            mpu9250.mag_raw.mag_zl = dat; raw_data[19] = dat;
            mpu9250.mag_raw.mag_Z =
                    (((short)mpu9250.mag_raw.mag_zh) << 8)
                    | mpu9250.mag_raw.mag_zl;
            dat_count = 0;
            mag_msg.magnetic_field.z = mpu9250.mag_raw.mag_Z;
            step = CRC_CHECK;
        }
        break;

    case CRC_CHECK:
#ifdef JUMP_SHOW
        cout << "go to CRC_CHECK" << endl;
#endif
        if(dat_count == 0)
        {
            crc_l = dat;
            dat_count++;
        }
        else if(dat_count == 1)
        {
            crc_h = dat;
            crc_res = ((short)crc_h << 8) | crc_l;
            dat_count = 0;
            crc_gen = CRC16(raw_data,20);
#ifdef CRC_SHOW
            cout << "crc_res " << crc_res << "--crc_gen " << crc_gen <<endl;
#endif
            if (crc_gen != crc_res)
            {
                step = CRC_ERR;
            }
            else
            {
#ifdef SENSOR_FILTER
                step = FRAME_FILTER;
#else
#ifdef SENSOR_SHOW
                step = FRAME_SHOW;
#else
                step = FRAME_END;
#endif
#endif
            }
        }
        break;

    case FRAME_FILTER:
#ifdef JUMP_SHOW
        cout << "go to FRAME_FILTER" << endl;
#endif



#ifdef SENSOR_SHOW
        step = FRAME_SHOW;
#else
        step = FRAME_END;
#endif
        break;

    case FRAME_SHOW:
#ifdef JUMP_SHOW
        cout << "go to FRAME_SHOW" << endl;
#endif

        //        cout<< "accel_X " << mpu9250.accel_raw.accel_X << "  "
        //               "accel_Y " << mpu9250.accel_raw.accel_Y << "  "
        //               "accel_Z " << mpu9250.accel_raw.accel_Z << endl;

        //        cout<< "gyro_X " << mpu9250.gyro_raw.gyro_X << "  "
        //               "gyro_Y " << mpu9250.gyro_raw.gyro_Y << "  "
        //               "gyro_Z " << mpu9250.gyro_raw.gyro_Z << endl;

        //        cout<< "mag_X " << mpu9250.mag_raw.mag_X << "  "
        //               "mag_Y " << mpu9250.mag_raw.mag_Y << "  "
        //               "mag_Z " << mpu9250.mag_raw.mag_Z << endl;

        cout << imu_msg.linear_acceleration.x << ","
             << imu_msg.linear_acceleration.y << ","
             << imu_msg.linear_acceleration.z << ","
             << imu_msg.angular_velocity.x    << ","
             << imu_msg.angular_velocity.y    << ","
             << imu_msg.angular_velocity.z    << endl;


        step = FRAME_END;
        break;


    case FRAME_END:
#ifdef JUMP_SHOW
        cout << "go to FRAME_END" << endl;
#endif

        if(dat == 0xcc || dat == 0xdd)
        {
            dat_count++;
        }
        else
        {
            dat_count = 0;
        }
        if(dat_count == 2)
        {
            dat_count = 0;

            imu_msg.orientation.w   = 1;

            imu_msg.header.frame_id = "mpu_9250";
            imu_msg.header.seq      = frame_num;
            imu_msg.header.stamp    = ros::Time::now();
            imu_pub.publish(imu_msg);
#ifdef FRAME_COUNTER
            cout << "Total frame receive: " << frame_num << endl;
#endif
            frame_num++;
            step = FRAME_BEGIN;
#ifdef JUMP_SHOW
            cout << "go to FRAME_BEGIN" << endl;
#endif

        }

        break;

    case CRC_ERR:
#ifdef JUMP_SHOW
        cout << "go to CRC_ERR" << endl;
#endif
        error_frame_num++;
        ROS_ERROR("CRC16 CHECK ERROR, JUMP TO FRAME BEGIN!");
        if(frame_num != 0)
            cout<<"Frame error rate " << (float)100*(float)error_frame_num/(float)frame_num <<"% ["<<error_frame_num<<"/"<<frame_num<<"]"<<endl;
        else
            cout<<"Frame error rate " << 0 <<"["<<error_frame_num<<"/"<<frame_num<<"]"<<endl;
        step = FRAME_BEGIN;
        break;

    }




}


void usage()
{
    printf( "This is a imu serial data decode programm. \n"
            "Usage: IMU Serial Decode\n"
            "       -p  <serial device>        #Serial Device path \"default </dev/ttyUSB0>\"\n"
            "       -b  <band speed>           #Serial Device band speed \"default <115200>\"\n"
            "       -x  <-TODO->               #Advance Parameter still not available     \"\n"
            "       -h  <help>                 #Show this useage                          \"\n"
            );

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "imu_data_decode");
    ros::NodeHandle n;
    imu_pub = n.advertise<sensor_msgs::Imu>("ahrs_data",50);


    //    ros::Rate loop_rate(100);
    //    string s1 = "abcdeg";
    //    const char *k = s1.c_str();
    //    const char *t = s1.data();

    //    std::string str = "string";
    //    char *cstr = new char[str.length() + 1];
    //    strcpy(cstr, str.c_str());
    //    // do stuff
    //    delete [] cstr;

    int band_speed = 115200;
    string device_s = "/dev/ttyUSB0";
    char * device = new char[device_s.length() + 1];
    strcpy(device, device_s.c_str());

    if(argc <= 1)
    {
        usage();
        return 0;
    }
    /**************************************************************************
     * get parameter from command line.
     *
     * ***********************************************************************/
    for(int i = 1; i < argc; i++)
    {
        const char* s = argv[i];


        if(strcmp( s, "-p" ) == 0)
        {
            device = argv[++i];
        }
        else if(strcmp( s, "-b" ) == 0)
        {
            char * ss = argv[++i];
            band_speed = atoi(ss);

        }
        else if(strcmp( s, "-h" ) == 0)
        {
            usage();
        }
        else
        {
            printf("Unsupport parameter.\n");
            usage();
        }
    }
    cout<< "select port "<< device << endl;
    fd = UART_Open(fd, device);
    if(UART_Set(fd, band_speed,0,8,1,'N') == FALSE)
    {
        cout << "port set fail" << endl;
    }

    step = FRAME_BEGIN;
    while(n.ok())
    {
        decode_loop();
        //ros::spinOnce();
        //        loop_rate.sleep();
    }
    UART_Close(fd);
    return 0;
}







