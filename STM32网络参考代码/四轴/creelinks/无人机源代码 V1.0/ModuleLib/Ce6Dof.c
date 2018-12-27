/**
  ******************************************************************************
  * @file    ce6Dof.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   ������ce6Dofģ����������ļ�
  ******************************************************************************
  * @attention
  *
  *1)MPU6050
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "Ce6Dof.h"
#include "math.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

// ����MPU6050�ڲ���ַ
//****************************************
#define    CE_6DOF_SMPLRT_DIV        0x19    //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define    CE_6DOF_CONFIG            0x1A    //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define    CE_6DOF_GYRO_CONFIG        0x1B    //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define    CE_6DOF_ACCEL_CONFIG    0x1C    //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define    CE_6DOF_ACCEL_XOUT_H    0x3B
#define    CE_6DOF_ACCEL_XOUT_L    0x3C
#define    CE_6DOF_ACCEL_YOUT_H    0x3D
#define    CE_6DOF_ACCEL_YOUT_L    0x3E
#define    CE_6DOF_ACCEL_ZOUT_H    0x3F
#define    CE_6DOF_ACCEL_ZOUT_L    0x40
#define    CE_6DOF_TEMP_OUT_H        0x41
#define    CE_6DOF_TEMP_OUT_L        0x42
#define    CE_6DOF_GYRO_XOUT_H        0x43
#define    CE_6DOF_GYRO_XOUT_L        0x44    
#define    CE_6DOF_GYRO_YOUT_H        0x45
#define    CE_6DOF_GYRO_YOUT_L        0x46
#define    CE_6DOF_GYRO_ZOUT_H        0x47
#define    CE_6DOF_GYRO_ZOUT_L        0x48
#define    CE_6DOF_PWR_MGMT_1        0x6B    //��Դ��������ֵ��0x00(��������)
#define    CE_6DOF_WHO_AM_I        0x75    //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define    CE_6DOF_ADDRESS    0xD0    //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

/**
  * @brief  Write an byte to the specified device address through I2C bus.
  * @param  ceI2cMaster:
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @param  Data: the data would be writen to the specified device address
  * @return  None
  */
void ce6Dof_I2C_WriteOneByte(CeI2cMaster *ceI2cMaster, uint8 DevAddr, uint8 RegAddr, uint8 Data)
{
    ceI2cMasterOp.start(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, DevAddr);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, RegAddr);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, Data);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.stop(ceI2cMaster);
}

/**
  * @brief Read an byte from the specified device address through I2C bus.
  * @param  ceI2cMaster:
  * @param DevAddr: The address byte of the slave device
  * @param RegAddr: The address byte of  register of the slave device
  * @return  the byte read from I2C bus
  */
uint8 ce6Dof_I2C_ReadOneByte(CeI2cMaster *ceI2cMaster, uint8 DevAddr, uint8 RegAddr)
{
    uint8 TempVal = 0;
    ceI2cMasterOp.start(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, DevAddr);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, RegAddr);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.start(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, DevAddr | 0x01);
    ceI2cMasterOp.start(ceI2cMaster);
    TempVal = ceI2cMasterOp.readByte(ceI2cMaster, 0x00);
    ceI2cMasterOp.stop(ceI2cMaster);
    return TempVal;
}

/**
  * @brief  ��ȡ���ٶ�
  * @param  ce6Dof:ce6Dof���Զ���
  * @return ���ٶ�����
  */
Ce6DofAcceleration *ce6Dof_getAcceleration(Ce6Dof *ce6Dof)
{
    uint8 tempBuf[6] = { 0 };
    tempBuf[0] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_ACCEL_XOUT_L);    
    tempBuf[1] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_ACCEL_XOUT_H);
    ce6Dof->acceleration.x = ((fp32)(((int16)(tempBuf[1]<<8)|tempBuf[0])) / 16384);  //���ٶ�X��

    tempBuf[2] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_ACCEL_YOUT_L);
    tempBuf[3] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_ACCEL_YOUT_H);
    ce6Dof->acceleration.y  = ((fp32)(((int16)(tempBuf[3]<<8)|tempBuf[2]))  / 16384); //���ٶ�Y��

    tempBuf[4] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_ACCEL_ZOUT_L);
    tempBuf[5] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_ACCEL_ZOUT_H);
    ce6Dof->acceleration.z = ((fp32)(((int16)(tempBuf[5]<<8)|tempBuf[4]))  / 16384);  //���ٶ�Z��
    return &(ce6Dof->acceleration);
}

/**
  * @brief  ��ȡ�����ǽ��ٶ�
  * @param  ce6Dof:ce6Dof���Զ���
  * @return ���ٶ�����
  */
Ce6DofGyroscope *ce6Dof_getGyroscope(Ce6Dof *ce6Dof)
{
    uint8 tempBuf[6] = { 0 };

    tempBuf[0] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_XOUT_L);
    tempBuf[1] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_XOUT_H);
    ce6Dof->gyroscope.x= ((fp32)(((int16)((tempBuf[1]& 0x00FF)<<8)|tempBuf[0]))) / 65.5;//������X��    

      tempBuf[2] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_YOUT_L);
    tempBuf[3] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_YOUT_H);
    ce6Dof->gyroscope.y =  ((fp32)(((int16)((tempBuf[3]& 0x00FF)<<8)|tempBuf[2]))) / 65.5;//������Y��    

    tempBuf[4] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_ZOUT_L);
    tempBuf[5] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_ZOUT_H);
    ce6Dof->gyroscope.z =  ((fp32)(((int16)((tempBuf[5]& 0x00FF)<<8)|tempBuf[4]))) / 65.5;//������Z��

    ce6Dof->gyroscope.x -= ce6Dof->gyroscopeZero.x;
    ce6Dof->gyroscope.y -= ce6Dof->gyroscopeZero.y;
    ce6Dof->gyroscope.z -= ce6Dof->gyroscopeZero.z;

    return &(ce6Dof->gyroscope);
}

/**
  * @brief  Ce6Dofģ���ʼ��
  * @param  ce6Dof:ce6Dof���Զ���
  */
CE_STATUS ce6Dof_initial(Ce6Dof *ce6Dof, CE_RESOURCE ceI2cMaster)
{
    ce6Dof->ceI2cMaster.ceResource = ceI2cMaster;
    ce6Dof->ceI2cMaster.ceI2cMasterSpeed = CE_I2C_SPEED_3_4MBPS;
    ceI2cMasterOp.initial(&(ce6Dof->ceI2cMaster));

    ce6Dof_I2C_WriteOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS,CE_6DOF_PWR_MGMT_1, 0x00);    //�������״̬
    ce6Dof_I2C_WriteOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS,CE_6DOF_SMPLRT_DIV, 0x07);
    ce6Dof_I2C_WriteOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS,CE_6DOF_CONFIG, 0x06);
    ce6Dof_I2C_WriteOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS,CE_6DOF_GYRO_CONFIG, 0x08);
    ce6Dof_I2C_WriteOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS,CE_6DOF_ACCEL_CONFIG, 0x01); 

    if(ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_WHO_AM_I) == 0x68)
    {
        int i;
        ceDebugOp.printf("MPU6050 initial success!\n");
        ceSystemOp.delayMs(10);
        ce6Dof->gyroscopeZero.x = 0;
        ce6Dof->gyroscopeZero.y = 0;
        ce6Dof->gyroscopeZero.z = 0;

        for(i = 0; i < 32; i++)
        {
            uint8 tempBuf[6] = {0};
            tempBuf[0] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_XOUT_L);
            tempBuf[1] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_XOUT_H);
            ce6Dof->gyroscope.x = (fp32)(((int16)(tempBuf[1]<<8)|tempBuf[0])) / 65.5;

            tempBuf[2] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_YOUT_L);
            tempBuf[3] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_YOUT_H);
            ce6Dof->gyroscope.y = (fp32)(((int16)(tempBuf[3]<<8)|tempBuf[2])) / 65.5;

            tempBuf[4] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_ZOUT_L);
            tempBuf[5] = ce6Dof_I2C_ReadOneByte(&(ce6Dof->ceI2cMaster), CE_6DOF_ADDRESS, CE_6DOF_GYRO_ZOUT_H);
            ce6Dof->gyroscope.z =  (fp32)(((int16)(tempBuf[5]<<8)|tempBuf[4])) / 65.5;

            ce6Dof->gyroscopeZero.x += ce6Dof->gyroscope.x;
            ce6Dof->gyroscopeZero.y += ce6Dof->gyroscope.y;
            ce6Dof->gyroscopeZero.z += ce6Dof->gyroscope.z;
            ceSystemOp.delayMs(1);
        }

        ce6Dof->gyroscopeZero.x /= 32;
        ce6Dof->gyroscopeZero.y /= 32;
        ce6Dof->gyroscopeZero.z /= 32;

        return CE_STATUS_SUCCESS;
    }
    else
    {
        ceDebugOp.printf("MPU6050 initial faile!\n");
        return CE_STATUS_FAILE;
    }
}

const Ce6DofOp ce6DofOp = {ce6Dof_initial, ce6Dof_getAcceleration, ce6Dof_getGyroscope};

#ifdef __cplusplus
}
#endif //__cplusplus

