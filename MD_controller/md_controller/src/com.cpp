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
            printf("send data : ");
            for(int k = 0; k < byPidDataSize; k++) 
            {
                printf("%d ", Com.bySndBuf[k]); 
            }
            printf("\n");
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
            // printf("send data : ");
            // for(int k = 0; k < byPidDataSize; k++) printf("%d ", Com.bySndBuf[k]); cout << endl;
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
    BYTE byRcvPID;

    // byRcvRMID     = Com.byRcvBuf[0];
    // byRcvTMID     = Com.byRcvBuf[1];
    // byRcvID       = Com.byRcvBuf[2];
    byRcvPID      = Com.byRcvBuf[3];
    // byRcvDataSize = Com.byRcvBuf[4];
    // printf("in!\n");
    switch(byRcvPID)
    {
        case PID_MAIN_DATA:
            Com.rpm  = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);

            Com.position = Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]);
            // printf("%d\n",Com.rpm);
            break;
    }

    return SUCCESS;
}

int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum) //Analyze the communication data
{
    static BYTE byChkSec;
    BYTE i, j;
    int count = 0;
    // printf("0 : %d , 1 : %d \n",byArray[0],byArray[1]);
    // printf("id : %d \n",byArray[2]);

    if(Com.byPacketNum >= MAX_PACKET_SIZE)
    {
        Com.byStep =0;
        return FAIL;
    }
    for(j = 0; j < byBufNum; j++)
    {
        switch(Com.byStep){
            case 0:    //Put the transmitting machin id after checking the data
                if((byArray[j] == 184) || (byArray[j] == 183))
                {
                    Com.byChkSum += byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                    Com.byChkComError = 0;
                    count++;
                    if(count == 2) Com.byStep++;
                }
                else
                {
                    printf("ERROR (1)\n");
                    count = 0;
                    Com.byStep      = 0;
                    Com.fgPacketOK  = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;

                }
                break;

            case 1:    //Check ID
                if(byArray[j] == 1 || byArray[j] == 2)
                {
                    Com.byChkSum += byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                    Com.byStep++;
                    Com.byChkComError = 0;
                }
                else
                {
                    printf("ERROR (2)\n");
                    Com.byStep = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;
                }
                break;

             case 2:    //Put the PID number into the array
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                Com.byStep++;
                break;

             case 3:    //Put the DATANUM into the array
                Com.byMaxDataNum = byArray[j];
                Com.byDataNum = 0;
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                Com.byStep++;
                break;

             case 4:    //Put the DATA into the array
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                Com.byChkSum += byArray[j];

                if(++Com.byDataNum >= MAX_DATA_SIZE)
                {
                    printf("check 5\n");
                    Com.byStep = 0;
                    Com.byTotalRcvDataNum = 0;
                    break;
                }

                if(Com.byDataNum>= Com.byMaxDataNum) Com.byStep++;
                break;

             case 5:    //Put the check sum after Checking checksum
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                // printf("byChkSum : %d \n", Com.byChkSum);
                if(Com.byChkSum == 0)
                {
                    Com.fgPacketOK   = 1;
                    Com.fgComDataChk = 1;
                    Com.byDataNum    = 0;
                    Com.byMaxDataNum = 0;
                }

                Com.byStep = 0;
                Com.byTotalRcvDataNum = 0;
                
                break;

            default:
                printf("check default\n");

                Com.byStep = 0;
                Com.fgComComple = ON;
                break;
        }
        if(Com.fgPacketOK)
        {

            Com.fgPacketOK   = 0;
            Com.byPacketSize = 0;
            Com.byPacketNum  = 0;

            if(byChkSec == 0)
            {
                byChkSec = 1;
            }
            MdReceiveProc();                                 //save the identified serial data to defined variable
        }

        if(Com.byChkComError == 10) //while 50ms
        {
            printf("check error\n");
    
            Com.byChkComError = 0;
            Com.byStep = 0;
            Com.byChkSum = 0;
            Com.byMaxDataNum = 0;
            Com.byDataNum = 0;
            for(i = 0; i < MAX_PACKET_SIZE; i++) Com.byRcvBuf[i] = 0;
            j = byBufNum;
        }

    }
    return SUCCESS;
}

int ReceiveDataFromController(BYTE init) //Analyze the communication data
{
    BYTE byRcvBuf[250];
    BYTE byBufNumber = 0;
    
    // LibSerial의 IsDataAvailable 메서드 사용
    if(serialPort.IsDataAvailable())
    {
        // 바이트 단위로 읽어서 배열에 저장
        try {
            while(serialPort.IsDataAvailable() && byBufNumber < MAX_DATA_SIZE) {
                serialPort.ReadByte(byRcvBuf[byBufNumber]);
                byBufNumber++;
            }
        } catch(const std::exception& e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error reading from serial port: " << e.what());
        }
        
        if(init == ON){
            if(byRcvBuf[2] == Motor.ID){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ID %d Motor Init success!", Motor.ID);
                Motor.InitMotor=OFF;
            }
        }
        else{
            AnalyzeReceivedData(byRcvBuf, byBufNumber);
        }
    }
    return 1;
}