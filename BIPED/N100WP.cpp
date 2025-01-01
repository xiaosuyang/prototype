// #include "N100WP.h"

// extern IMUData_Packet_t IMUData_Packet;
// extern AHRSData_Packet_t AHRSData_Packet;
// extern int rs_imutype;
// extern int rs_ahrstype;
// extern u8 Fd_rsimu[64];
// extern u8 Fd_rsahrs[56];

// static const uint8_t CRC8Table[] = {
//     0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65, 157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255, 70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36, 248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
//     175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115, 202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

// static const uint16_t CRC16Table[256] =
//     {
//         0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
//         0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};

// uint8_t CRC8_Tabletest(uint8_t *p, uint8_t counter)
// {
//     uint8_t crc8 = 0;
//     for (int i = 0; i < counter; i++)
//     {
//         uint8_t value = p[i];
//         uint8_t new_index = crc8 ^ value;
//         crc8 = CRC8Table[new_index];
//     }
//     return (crc8);
// }

// uint16_t CRC16_Tabletest(uint8_t *p, uint8_t counter)
// {
//     uint16_t crc16 = 0;
//     for (int i = 0; i < counter; i++)
//     {
//         uint8_t value = p[i];
//         crc16 = CRC16Table[((crc16 >> 8) ^ value) & 0xff] ^ (crc16 << 8);
//     }
//     return (crc16);
// }

// float DATA_Trans(u8 Data_1, u8 Data_2, u8 Data_3, u8 Data_4)
// {
//     u32 transition_32;
//     float tmp = 0;
//     int sign = 0;
//     int exponent = 0;
//     float mantissa = 0;
//     transition_32 = 0;
//     transition_32 |= Data_4 << 24;
//     transition_32 |= Data_3 << 16;
//     transition_32 |= Data_2 << 8;
//     transition_32 |= Data_1;
//     sign = (transition_32 & 0x80000000) ? -1 : 1; //
//     //
//     exponent = ((transition_32 >> 23) & 0xff) - 127;
//     //
//     mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
//     tmp = sign * mantissa * pow(2, exponent);
//     return tmp;
// }

// long long timestamp(u8 Data_1, u8 Data_2, u8 Data_3, u8 Data_4)
// {
//     u32 transition_32;
//     transition_32 = 0;
//     transition_32 |= Data_4 << 24;
//     transition_32 |= Data_3 << 16;
//     transition_32 |= Data_2 << 8;
//     transition_32 |= Data_1;
//     return transition_32;
// }

// u8 TTL_Hex2Dec()
// {
//     if (rs_ahrstype == 1)
//     {
//         if (Fd_rsahrs[1] == TYPE_AHRS && Fd_rsahrs[2] == AHRS_LEN)
//         {
//             AHRSData_Packet.RollSpeed = DATA_Trans(Fd_rsahrs[7], Fd_rsahrs[8], Fd_rsahrs[9], Fd_rsahrs[10]);       // ºá¹ö½ÇËÙ¶È
//             AHRSData_Packet.PitchSpeed = DATA_Trans(Fd_rsahrs[11], Fd_rsahrs[12], Fd_rsahrs[13], Fd_rsahrs[14]);   // ¸©Ñö½ÇËÙ¶È
//             AHRSData_Packet.HeadingSpeed = DATA_Trans(Fd_rsahrs[15], Fd_rsahrs[16], Fd_rsahrs[17], Fd_rsahrs[18]); // Æ«º½½ÇËÙ¶È

//             AHRSData_Packet.Roll = DATA_Trans(Fd_rsahrs[19], Fd_rsahrs[20], Fd_rsahrs[21], Fd_rsahrs[22]);    // ºá¹ö½Ç
//             AHRSData_Packet.Pitch = DATA_Trans(Fd_rsahrs[23], Fd_rsahrs[24], Fd_rsahrs[25], Fd_rsahrs[26]);   // ¸©Ñö½Ç
//             AHRSData_Packet.Heading = DATA_Trans(Fd_rsahrs[27], Fd_rsahrs[28], Fd_rsahrs[29], Fd_rsahrs[30]); // Æ«º½½Ç

//             AHRSData_Packet.Qw = DATA_Trans(Fd_rsahrs[31], Fd_rsahrs[32], Fd_rsahrs[33], Fd_rsahrs[34]); // ËÄÔªÊý
//             AHRSData_Packet.Qx = DATA_Trans(Fd_rsahrs[35], Fd_rsahrs[36], Fd_rsahrs[37], Fd_rsahrs[38]);
//             AHRSData_Packet.Qy = DATA_Trans(Fd_rsahrs[39], Fd_rsahrs[40], Fd_rsahrs[41], Fd_rsahrs[42]);
//             AHRSData_Packet.Qz = DATA_Trans(Fd_rsahrs[43], Fd_rsahrs[44], Fd_rsahrs[45], Fd_rsahrs[46]);
//             AHRSData_Packet.Timestamp = timestamp(Fd_rsahrs[47], Fd_rsahrs[48], Fd_rsahrs[49], Fd_rsahrs[50]); // Ê±¼ä´Á
//             //	AHRSData2PC();
//         }
//         rs_ahrstype = 0;
//     }
//     if (rs_imutype == 1)
//     {
//         if (Fd_rsimu[1] == TYPE_IMU && Fd_rsimu[2] == IMU_LEN)
//         {
//             IMUData_Packet.gyroscope_x = DATA_Trans(Fd_rsimu[7], Fd_rsimu[8], Fd_rsimu[9], Fd_rsimu[10]); // ½ÇËÙ¶È
//             IMUData_Packet.gyroscope_y = DATA_Trans(Fd_rsimu[11], Fd_rsimu[12], Fd_rsimu[13], Fd_rsimu[14]);
//             IMUData_Packet.gyroscope_z = DATA_Trans(Fd_rsimu[15], Fd_rsimu[16], Fd_rsimu[17], Fd_rsimu[18]);

//             IMUData_Packet.accelerometer_x = DATA_Trans(Fd_rsimu[19], Fd_rsimu[20], Fd_rsimu[21], Fd_rsimu[22]); // Ïß¼ÓËÙ¶È
//             IMUData_Packet.accelerometer_y = DATA_Trans(Fd_rsimu[23], Fd_rsimu[24], Fd_rsimu[25], Fd_rsimu[26]);
//             IMUData_Packet.accelerometer_z = DATA_Trans(Fd_rsimu[27], Fd_rsimu[28], Fd_rsimu[29], Fd_rsimu[30]);

//             // IMUData_Packet.magnetometer_x=DATA_Trans(Fd_rsimu[31],Fd_rsimu[32],Fd_rsimu[33],Fd_rsimu[34]);  //´ÅÁ¦¼ÆÊý¾Ý
//             // IMUData_Packet.magnetometer_y=DATA_Trans(Fd_rsimu[35],Fd_rsimu[36],Fd_rsimu[37],Fd_rsimu[38]);
//             // IMUData_Packet.magnetometer_z=DATA_Trans(Fd_rsimu[39],Fd_rsimu[40],Fd_rsimu[41],Fd_rsimu[42]);

//             IMUData_Packet.Timestamp = timestamp(Fd_rsimu[55], Fd_rsimu[56], Fd_rsimu[57], Fd_rsimu[58]); // Ê±¼ä´Á
//             //   IMUData2PC();
//         }
//         rs_imutype = 0;
//     }
//     return 0;
// }
