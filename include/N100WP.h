// #ifndef N100WP
// #define N100WP

// #define FRAME_HEAD 0xfc
// #define FRAME_END 0xfd
// #define TYPE_IMU 0x40
// #define TYPE_AHRS 0x41
// #define TYPE_INSGPS 0x42
// #define TYPE_GROUND 0xf0
// #define IMU_LEN  0x38   //56+8  
// #define AHRS_LEN 0x30   //48+8  
// #define IMU_RS 64
// #define AHRS_RS 56

// #include "cTypes.h"
// #include <math.h>


// typedef struct IMUData_Packet_t{
// 		float gyroscope_x;          //unit: rad/s
// 		float gyroscope_y;          //unit: rad/s
// 		float gyroscope_z;          //unit: rad/s
// 		float accelerometer_x;      //m/s^2
// 		float accelerometer_y;      //m/s^2
// 		float accelerometer_z;      //m/s^2
// 		// float magnetometer_x;       //mG
// 		// float magnetometer_y;       //mG
// 		// float magnetometer_z;       //mG
// 		// float imu_temperature;      //C
// 		// float Pressure;             //Pa
// 		// float pressure_temperature; //C
// 		u32 Timestamp;          //us
// } IMUData_Packet_t;

// typedef struct AHRSData_Packet_t
// {
// 	float RollSpeed;   //unit: rad/s
// 	float PitchSpeed;  //unit: rad/s
// 	float HeadingSpeed;//unit: rad/s
// 	float Roll;        //unit: rad 
// 	float Pitch;       //unit: rad
// 	float Heading;     //unit: rad
// 	float Qw;//w          //Quaternion
// 	float Qx;//x
// 	float Qy;//y
// 	float Qz;//z
// 	u32 Timestamp; //unit: us
// }AHRSData_Packet_t;



// long long timestamp(u8 Data_1,u8 Data_2,u8 Data_3,u8 Data_4);

// u8 TTL_Hex2Dec(); 

// float DATA_Trans(u8 Data_1,u8 Data_2,u8 Data_3,u8 Data_4);


// uint8_t CRC8_Tabletest(uint8_t* p, uint8_t counter);

// uint16_t CRC16_Tabletest(uint8_t* p, uint8_t counter);


// #endif