#include "md/com.hpp"
#include "md/global.hpp"
#include "robot.cpp"

#define MD_PROTOCOL_POS_PID 3
#define MD_PROTOCOL_POS_DATA_LEN 4
#define MD_PROTOCOL_POS_DATA_START 5

serial::Serial ser;

PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
PID_PNT_IO_MONITOR_t curr_pid_pnt_io_monitor;
PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
PID_GAIN_t curr_pid_gain;

uint8_t serial_comm_rcv_buff[MAX_PACKET_SIZE];
uint8_t serial_comm_snd_buff[MAX_PACKET_SIZE];

extern INIT_SETTING_STATE_t fgInitsetting;
// extern int CalRobotPoseFromRPM(PID_PNT_MAIN_DATA_t *pData);
// extern int CalRobotPoseFromPos(PID_PNT_MAIN_DATA_t *pData);
extern void PubRobotPose(void);

// Initialize serial communication in ROS
int InitSerialComm(void)
{
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(robotParamData.nBaudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1667); // 1667 when baud is 57600, 0.6ms
        ser.setTimeout(to);                                        // 2857 when baud is 115200, 0.35ms
        ser.open();
    }

    catch (serial::IOException& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Serial"), "Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        RCLCPP_INFO(rclcpp::get_logger("SerIsOpen"), "Serial Port initialized");
        return 1;
    }
    else {
        return -1;
    }
}

uint8_t CalCheckSum(uint8_t* pData, uint16_t length)
{
    uint8_t sum;

    sum = 0;
    for (int i = 0; i < length; i++) {
        sum += *pData++;
    }

    sum = ~sum + 1; // check sum

    return sum;
}

int PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t* pData, uint16_t length)
{
    uint8_t* p;
    uint16_t len;

    len = 0;
    serial_comm_snd_buff[len++] = rmid;
    serial_comm_snd_buff[len++] = robotParamData.nIDPC;
    serial_comm_snd_buff[len++] = 1;
    serial_comm_snd_buff[len++] = (uint8_t)pid;
    serial_comm_snd_buff[len++] = length;

    p = (uint8_t*)&serial_comm_snd_buff[len];
    memcpy((char*)p, (char*)pData, length);
    len += length;

    serial_comm_snd_buff[len++] = CalCheckSum(serial_comm_snd_buff, len);

    if (ser.isOpen() == true) {
        // ser.flushInput();
        ser.write(serial_comm_snd_buff, len);
    }

    return 1;
}

int MdReceiveProc(void) // save the identified serial data to defined variable according to PID NUMBER data
{
    uint8_t* pRcvBuf;
    uint8_t* pRcvData;
    uint8_t byRcvPID;
    uint8_t byRcvDataSize;

    pRcvBuf = serial_comm_rcv_buff;

    byRcvPID = pRcvBuf[MD_PROTOCOL_POS_PID];
    byRcvDataSize = pRcvBuf[MD_PROTOCOL_POS_DATA_LEN];
    pRcvData = &pRcvBuf[MD_PROTOCOL_POS_DATA_START];

    switch (byRcvPID) {
    case PID_GAIN: {
        if (byRcvDataSize == sizeof(PID_GAIN_t)) {
            memcpy((char*)&curr_pid_gain, (char*)pRcvData, sizeof(PID_GAIN_t));
        }
        break;
    }

    case PID_PNT_MAIN_DATA: // 210
    {
        if (fgInitsetting != INIT_SETTING_STATE_OK) {
            break;
        }

        if (byRcvDataSize == sizeof(PID_PNT_MAIN_DATA_t)) {
            memcpy((char*)&curr_pid_pnt_main_data, (char*)pRcvData, sizeof(PID_PNT_MAIN_DATA_t));

            // CalRobotPoseFromRPM(&curr_pid_pnt_main_data);
            CalRobotPoseFromPos(&curr_pid_pnt_main_data);

            PubRobotPose();
        }
        break;
    }

    case PID_ROBOT_PARAM: // 247
    {
        if (byRcvDataSize == sizeof(PID_ROBOT_PARAM_t)) {
            PID_ROBOT_PARAM_t* p;

            p = (PID_ROBOT_PARAM_t*)pRcvData;
            robotParamData.sSetDia = p->nDiameter;         // mm unit
            robotParamData.sSetWheelLen = p->nWheelLength; // mm unit
            robotParamData.sSetGear = p->nGearRatio;
        }
        break;
    }

    case PID_ROBOT_MONITOR2: // 224
    {
        if (byRcvDataSize == sizeof(PID_ROBOT_MONITOR2_t)) {
            memcpy((char*)&curr_pid_robot_monitor2, (char*)pRcvData, sizeof(PID_ROBOT_MONITOR2_t));
        }
        break;
    }

    case PID_PNT_IO_MONITOR: // 241
    {
        if (byRcvDataSize == sizeof(PID_PNT_IO_MONITOR_t)) {
            memcpy((char*)&curr_pid_pnt_io_monitor, (char*)pRcvData, sizeof(PID_PNT_IO_MONITOR_t));
        }
        break;
    }
    }
    return 1;
}

int AnalyzeReceivedData(uint8_t byArray[], uint8_t byBufNum) // Analyze the communication data
{
    // rclcpp::NodeHandle n;
    // ros::Time stamp;
    uint8_t i, j;
    uint8_t data;
    static uint8_t byChkSec;
    static long lExStampSec, lExStampNsec;
    static uint32_t byPacketNum;
    static uint32_t rcv_step;
    static uint8_t byChkSum;
    static uint16_t byMaxDataNum;
    static uint16_t byDataNum;

    if (byPacketNum >= MAX_PACKET_SIZE) {
        rcv_step = 0;
        byPacketNum = 0;

        return 0;
    }

    for (j = 0; j < byBufNum; j++) {
        data = byArray[j];

        switch (rcv_step) {
        case 0: // Put the reading machin id after checking the data
            if (data == robotParamData.nIDPC) {
                byPacketNum = 0;
                byChkSum = data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                rcv_step++;
            }
            else {
                byPacketNum = 0;
            }
            break;
        case 1: // Put the transmitting machin id after checking the data
            if ((data == robotParamData.nIDMDUI) || (data == robotParamData.nIDMDT)) {
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                rcv_step++;
            }
            else {
                rcv_step = 0;
                byPacketNum = 0;
            }
            break;

        case 2: // Check ID
            if (data == 1 || data == ID_ALL) {
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                rcv_step++;
            }
            else {
                rcv_step = 0;
                byPacketNum = 0;
            }
            break;
        case 3: // Put the PID number into the array
            byChkSum += data;
            serial_comm_rcv_buff[byPacketNum++] = data;

            rcv_step++;
            break;

        case 4: // Put the DATANUM into the array
            byChkSum += data;
            serial_comm_rcv_buff[byPacketNum++] = data;

            byMaxDataNum = data;
            byDataNum = 0;

            rcv_step++;
            break;

        case 5: // Put the DATA into the array
            byChkSum += data;
            serial_comm_rcv_buff[byPacketNum++] = data;

            if (++byDataNum >= MAX_DATA_SIZE) {
                rcv_step = 0;
                break;
            }

            if (byDataNum >= byMaxDataNum) {
                rcv_step++;
            }
            break;

        case 6: // Put the check sum after Checking checksum
            byChkSum += data;
            serial_comm_rcv_buff[byPacketNum++] = data;

            if (byChkSum == 0) {
                MdReceiveProc(); // save the identified serial data to defined variable
            }

            rcv_step = 0;
            break;

        default: rcv_step = 0; break;
        }
    }
    return 1;
}

int ReceiveDataFromController(void) // Analyze the communication data
{
    uint8_t byRcvBuf[250];
    uint8_t byBufNumber;

    byBufNumber = ser.available();
    if (byBufNumber != 0) {
        if (byBufNumber > sizeof(byRcvBuf)) {
            byBufNumber = sizeof(byRcvBuf);
        }

        ser.read(byRcvBuf, byBufNumber);
        AnalyzeReceivedData(byRcvBuf, byBufNumber);
    }

    return 1;
}
