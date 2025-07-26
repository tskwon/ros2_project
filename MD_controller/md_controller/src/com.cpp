#include "md_controller/com.hpp"
#include <libserial/SerialPort.h>
#include <chrono>
#include <thread>

// serial::Serial 대신 LibSerial::SerialPort 사용
LibSerial::SerialPort serialPort;
Communication Com;
MotorVar Motor;

// Get the low and high byte from short
IByte Short2Byte(short sIn)
{
    IByte Ret;

    Ret.byLow = sIn & 0xff;
    Ret.byHigh = sIn>>8 & 0xff;

    return Ret;
}

// Make short data from two bytes
int Byte2Short(BYTE byLow, BYTE byHigh)
{
    return (byLow | (int)byHigh<<8);
}

// Make long data from four bytes
int Byte2LInt(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4)
{
    return ((int)byData1 | (int)byData2<<8 | (int)byData3<<16 | (int)byData4<<24);
}
//Initialize serial communication in ROS
int InitSerial(void)
{
    try
    {
        // 포트 열기
        serialPort.Open(Com.nPort);
        
        // 보드레이트 설정
        switch(Com.nBaudrate)
        {
            case 9600:
                serialPort.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
                break;
            case 19200:
                serialPort.SetBaudRate(LibSerial::BaudRate::BAUD_19200);
                break;
            case 38400:
                serialPort.SetBaudRate(LibSerial::BaudRate::BAUD_38400);
                break;
            case 57600:
                serialPort.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
                break;
            case 115200:
                serialPort.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
                break;
            default:
                serialPort.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
                break;
        }
        
        // 다른 시리얼 설정
        serialPort.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serialPort.SetParity(LibSerial::Parity::PARITY_NONE);
        serialPort.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serialPort.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        
        // 타임아웃 설정 (LibSerial에서는 다른 방식으로 처리)
        // VTime: 문자 간 타임아웃 (단위: 0.1초)
        serialPort.SetVTime(10); // 1초 타임아웃
    }
    catch (const LibSerial::OpenFailed& e)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Unable to open port: " << e.what());
        return -1;
    }
    
    if(serialPort.IsOpen())
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Serial Port initialized");
    else
        return -1;
    
    return 0;
}


//for sending the data (One ID)
int PutMdData(BYTE byPID, BYTE byMID, int id_num, int nArray[])
{
    // IByte iData;
    BYTE byPidDataSize, byDataSize, i, j;
    static BYTE byTempDataSum;
    
    for(j = 0; j <MAX_PACKET_SIZE; j++) Com.bySndBuf[j] = 0;

    Com.bySndBuf[0] = byMID;
    Com.bySndBuf[1] = 184;
    Com.bySndBuf[2] = id_num;
    Com.bySndBuf[3] = byPID;

    switch(byPID)
    {
        case PID_REQ_PID_DATA:
                byDataSize      = 1;
                byPidDataSize   = 7;
                byTempDataSum   = 0;

                Com.bySndBuf[4] = byDataSize;
                Com.bySndBuf[5] = (BYTE)nArray[0];

                for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
                Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

                // LibSerial::Write는 길이를 두 번째 인자로 받지 않음
                serialPort.Write(std::string(reinterpret_cast<char*>(Com.bySndBuf), byPidDataSize));

                break;
                
        case PID_POSI_RESET:
                byDataSize    = 1;
                byPidDataSize = 7;
                byTempDataSum = 0;

                Com.bySndBuf[4]  = byDataSize;
                Com.bySndBuf[5]  = nArray[0];

                for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
                Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

                serialPort.Write(std::string(reinterpret_cast<char*>(Com.bySndBuf), byPidDataSize));

                break;

        case PID_COMMAND:
            byDataSize    = 1;
            byPidDataSize = 7;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = nArray[0];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            serialPort.Write(std::string(reinterpret_cast<char*>(Com.bySndBuf), byPidDataSize));

            break;

        case PID_PNT_VEL_CMD:
            byDataSize    = 7;
            byPidDataSize = 13;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = 1;
            Com.bySndBuf[6]  = nArray[0];
            Com.bySndBuf[7]  = nArray[1];
            Com.bySndBuf[8]  = 1;
            Com.bySndBuf[9]  = nArray[2];
            Com.bySndBuf[10]  = nArray[3];
            Com.bySndBuf[11]  = 0;

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            serialPort.Write(std::string(reinterpret_cast<char*>(Com.bySndBuf), byPidDataSize));
            
            break;
        
        case PID_PNT_TQ_OFF:
            byDataSize    = 3;
            byPidDataSize = 9;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = 1;
            Com.bySndBuf[6]  = 1;
            Com.bySndBuf[7]  = 0;

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            serialPort.Write(std::string(reinterpret_cast<char*>(Com.bySndBuf), byPidDataSize));
            
            break;
                
        case PID_VEL_CMD:
            byDataSize    = 2;
            byPidDataSize = 8;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = nArray[0];
            Com.bySndBuf[6]  = nArray[1];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            serialPort.Write(std::string(reinterpret_cast<char*>(Com.bySndBuf), byPidDataSize));
            // printf("send data : ");
            // for(int k = 0; k < byPidDataSize; k++) printf("%d ", Com.bySndBuf[k]); cout << endl;
            break;
    }
    
    return SUCCESS;
}


int MdReceiveProc(void)
{
    BYTE *pRcvBuf;
    BYTE *pRcvData;
    BYTE byRcvPID;
    BYTE byRcvDataSize;

    pRcvBuf = Com.byRcvBuf;

    byRcvPID      = pRcvBuf[3];   
    byRcvDataSize = pRcvBuf[4];   
    pRcvData      = &pRcvBuf[5];  

    // printf("RCV PID: %d, Size: %d\n", byRcvPID, byRcvDataSize);

    switch(byRcvPID)
    {
        case PID_MAIN_DATA: // 193
        {           
            break;
        }

        case 216: // PID_PNT_MONITOR
        {
            if(byRcvDataSize == 14) {
                // // Raw 바이트 출력 (십진수)
                // printf("Monitor raw bytes: ");
                // for(int i = 0; i < 14; i++) {
                //     printf("%d ", pRcvData[i]);
                // }
                // printf("\n");

                // 모터1 데이터
                int motor1_rpm = Byte2Short(pRcvData[0], pRcvData[1]);     // D1,D2
                BYTE motor1_status = pRcvData[2];                          // D3
                int motor1_position = Byte2LInt(pRcvData[3], pRcvData[4], 
                                              pRcvData[5], pRcvData[6]);   // D4,D5,D6,D7
                
                // 모터2 데이터  
                int motor2_rpm = Byte2Short(pRcvData[7], pRcvData[8]);     // D8,D9
                BYTE motor2_status = pRcvData[9];                          // D10
                int motor2_position = Byte2LInt(pRcvData[10], pRcvData[11], 
                                              pRcvData[12], pRcvData[13]); // D11,D12,D13,D14
                
                // 전역 변수에 저장
                Motor.right_position = motor1_position;
                Motor.left_position = motor2_position;

                // printf("Motor1 - RPM: %d, Status: %d, Position: %d\n", 
                //        motor1_rpm, motor1_status, motor1_position);
                // printf("Motor2 - RPM: %d, Status: %d, Position: %d\n", 
                //        motor2_rpm, motor2_status, motor2_position);
            }
            break;
        }

        default:
            // printf("Unknown PID: %d\n", byRcvPID);
            break;
    }

    return 1;
}


int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum)
{
    BYTE i, j;
    BYTE data;
    static BYTE byChkSec;
    static uint32_t byPacketNum;
    static uint32_t rcv_step;
    static BYTE byChkSum;
    static uint16_t byMaxDataNum;
    static uint16_t byDataNum;

    // printf("Analyzing received data...\n");
    // for(i = 0; i < byBufNum; i++) {
    //     printf("%d ", byArray[i]);
    // }
    // printf("\n");
    if(byPacketNum >= MAX_PACKET_SIZE)
    {
        rcv_step = 0;
        byPacketNum = 0;
        return 0;
    }
    
    for(j = 0; j < byBufNum; j++)
    {
        data = byArray[j];
        
        switch(rcv_step) {
            case 0:    // PC ID 확인
                if(data == Com.nIDMDUI)  // 184
                {
                    byPacketNum = 0;
                    byChkSum = data;
                    Com.byRcvBuf[byPacketNum++] = data;
                    rcv_step++;
                }
                else
                {
                    byPacketNum = 0;
                }
                break;

            case 1:    // MDT/MDUI ID 확인
                if((data == Com.nIDMDUI) || (data == Com.nIDMDT))  // 184 or 183
                {
                    byChkSum += data;
                    Com.byRcvBuf[byPacketNum++] = data;
                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;
                }
                break;

            case 2:    // 모터 ID 확인
                if(data == 1 || data == 0xFE)  // ID 1 또는 ID_ALL
                {
                    byChkSum += data;
                    Com.byRcvBuf[byPacketNum++] = data;
                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;
                }
                break;

             case 3:    // PID 저장
                byChkSum += data;
                Com.byRcvBuf[byPacketNum++] = data;
                rcv_step++;
                break;

             case 4:    // 데이터 길이 저장
                byChkSum += data;
                Com.byRcvBuf[byPacketNum++] = data;
                
                byMaxDataNum = data;
                byDataNum = 0;
                rcv_step++;
                break;

             case 5:    // 데이터 저장
                byChkSum += data;
                Com.byRcvBuf[byPacketNum++] = data;

                if(++byDataNum >= MAX_DATA_SIZE)
                {
                    rcv_step = 0;
                    break;
                }

                if(byDataNum >= byMaxDataNum) {
                    rcv_step++;
                }
                break;

             case 6:    // 체크섬 확인
                byChkSum += data;
                Com.byRcvBuf[byPacketNum++] = data;

                if(byChkSum == 0)
                {
                    // 패킷 완성!
                    MdReceiveProc();
                }
                else {
                    printf("Checksum Error: %d\n", byChkSum);
                }

                byPacketNum = 0;
                rcv_step = 0;
                break;

            default:
                rcv_step = 0;
                break;
        }
    }
    return 1;
}
int ReceiveDataFromController(BYTE init) {
    BYTE byRcvBuf[250];
    BYTE byBufNumber = 0;

    // 수신 버퍼 초기화
    memset(byRcvBuf, 0, sizeof(byRcvBuf));

    if (serialPort.IsDataAvailable()) {
        try {
            // 시리얼 포트 입력 버퍼 비우기 (선택적)
            // serialPort.FlushIOBuffers(); // 주석 처리: 필요한 경우 활성화
            while (serialPort.IsDataAvailable() && byBufNumber < MAX_DATA_SIZE) {
                serialPort.ReadByte(byRcvBuf[byBufNumber]);
                byBufNumber++;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error reading from serial port: " << e.what());
        }

        if (init == ON) {
            if (byRcvBuf[2] == Motor.ID) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ID %d Motor Init success!", Motor.ID);
                Motor.InitMotor = OFF;
            }
        } else {
            AnalyzeReceivedData(byRcvBuf, byBufNumber);
        }
    }
    return 1;
}