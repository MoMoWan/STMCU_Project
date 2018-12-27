/**
  ******************************************************************************
  * @file    CeTask.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinksƽ̨CeTask��ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *1)���ǰ���޲���ϵͳ������㱼��ƽ̨������ע��������񣬾����������е�mainTask�����ڣ���ע��˳�����ڵ��ã�
  *  ���û����뱣֤mainTask������������������ע�����������callBack��Ҳ�����ǿɶ�ʱ���ڿ�ִ���겢���صġ�
  *2)����������ϵͳ������RTOS�������Ը���isNewThread��ָ���������Ƕ������񣬻����ں�̨�е��õ�����
  *3)�����������(isNewThread = 0x01)�����ʹ��RTOS�Ĵ����������������񴴽���Ϊ�˱�֤���������ܹ��������У�
  *  ����ע��ĺ������ʵ������ʱ������
  *4)����Ǻ�̨�е��õ�����Creelinksƽ̨��ע��ĺ���������ͬ���Ǻ�̨�����ע�ắ����ͳһ��һ����һ����ϵͳ
  *  ��������������ѭ���ã���ע��ĺ����о�����Ҫ����ʱ������
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_LOOP_H__
#define __CE_LOOP_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#ifdef __CE_USE_RTOS__
/**
  * @brief  RTOSģʽ�£���������ȼ��������ݶ�5�����ȼ�
  */
 typedef enum
 {
     CE_TASK_PRIORITY_HH = 0,
     CE_TASK_PRIORITY_H,
     CE_TASK_PRIORITY_M,
     CE_TASK_PRIORITY_L,
     CE_TASK_PRIORITY_LL,
 }CE_TASK_PRIORITY;
#endif //__CE_USE_RTOS__

/**
  * @brief  �ṹ�壬Task������ò�������
  */
typedef struct CeTaskBase
{
    uint32      ID;                                 /*!< ��ǰ�̵߳�ID��*/
    void*       pAddPar;                            /*!< �����ָ�룬ִ�лص�ʱ�����ָ��*/
    void        (*callBack)(void* pAddPar);         /*!< �̵߳���ѭ������*/

    #ifdef __CE_USE_RTOS__
    CE_TASK_PRIORITY taskPriority;                  /*!< �����������һ�����߳��н���������Ҫָ����������ȼ�*/
    CE_STK*     taskStackBuf;                       /*!< ָ������Ķ�ջ�׵�ַ*/
    uint32      taskStackBufSize;                   /*!< ָ�������ջ�Ĵ�С*/
    char*       taskName;                           /*!< ָ�����������*/
    #endif

    struct  CeTaskBase* nextCeTask;                 /*!< �������ڱ�����һ���߳�*/
    CeExTaskPar    ceExTaskPar;
}CeTask;

/**
 * @brief  �ṹ�壬Task������ò�������
 */
typedef struct
{

    CE_STATUS   (*mainTask)();                      /*!< @brief ��main��ں�������whileѭ���е��õ�����ѭ��������*/

    CE_STATUS   (*registerTask)(CeTask* ceTask);    /*!< @brief ע��һ���������е��߳�
                                                         @param ceTask:�̶߳����ָ��*/

    CE_STATUS   (*start)(CeTask* ceTask);           /*!< @brief ��ʼһ��ע�����߳�
                                                         @param ceTask:�̶߳����ָ��*/

    CE_STATUS   (*stop)(CeTask* ceTask);            /*!< @brief ֹͣһ��ע�����߳�
                                                         @param ceTask:�̶߳����ָ��*/

    CE_STATUS   (*unRegister)(CeTask* ceTask);      /*!< @brief ȡ���̵߳�ע��
                                                         @param ceTask:�̶߳����ָ��*/
  
    void        (*inCriticalSection)(void);         /*!<@brief �����ٽ�δ��룬�˺��ֹ�����л�*/

    void        (*outCriticalSection)(void);        /*!<@brief �����ٽ�δ��룬�˺������л�������������*/

    uint8       (*getCriticalStatus)(void);         /*!<@brief ��֤�Ƿ����ٽ�״̬
                                                        @return ����0x01:�������ٽ�ף�0x00��δ�����ٽ��*/

    void        (*taskSchedule)(void);              /*!< @brief ֪ͨ����ϵͳ�������������*/
} CeTaskOp;
extern const CeTaskOp ceTaskOp;                     /*!< ������Task��صĲ���*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_LOOP_H__


/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ǰ��̨�ǲ���ϵͳ���������㱼����)
* @function ��ʼһ������ÿ��ִ��ʱͨ��Uart����λ�����͵�����Ϣ
******************************************************************************
#include "Creelinks.h"
CeTask myTask;                                  //����Task���Զ���
void myTaskCallBack(void * pAddPar)             //������������ע�⣺���㱼ϵͳ�У��˺��������ܹ���ʱ��ִ����ϣ���һ������Ϊwhile(1)��ѭ��
{
    ceDebugOp.printf("Task ID %d is running!\n", ((CeTask*)(pAddPar))->ID);
}
int main(void)
{
    ceSystemOp.initial();                       //Creelinks������ʼ��
    ceDebugOp.initial(Uartx);                        //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ���Ȳ���             
    myTask.ID = 0x01;                           //ָ������ID   
    myTask.pAddPar = &myTask;                   //ָ��Task�����еĿ�ָ�룬����������ʱ����ص�����
    myTask.callBack = myTaskCallBack;           //ָ������Ļص�����
    ceTaskOp.registerTask(&myTask);             //ע�������
    ceTaskOp.start(&myTask);                    //��ʼ������
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks������ѭ�������뱣֤�˺����ܹ������ڵ���
        //TODO:���ڴ˴������û�����
        ceSystemOp.delayMs(200);                //��ʱ����
    };
}
******************************************************************************
*/

/**
******************************************************************************
* @brief  ʹ�����̼�ʾ������(����ʵʱ����ϵͳ(RTOS)����)
* @function �´����������������񣬲�����λ����ӡ������Ϣ
****************************************************************************** 
#include "Creelinks.h"
#define MY_TASK1_STACK_BUF_SIZE  1024                   //����1��ջ�����С
CE_STK MY_TASK1_STACK_BUF[MY_TASK1_STACK_BUF_SIZE];     //����1ʹ�ö�ջ����
CeTask myTask1;                                         //��������1�����Զ���ʵ��
void myTask1CallBack(void* pAddPar)                     //ʾ������1������ע�⣺��Creelinks���������񴴽��͵��ý��м򻯴��������������Բ���while(1)ѭ����Ҳ�ܱ�֤���ڵ���
{
    ceDebugOp.printf("Task 1 is running...\n");
    ceSystemOp.delayMs(1000);                           //�ʵ���ʱ���ô�����������������
}

#define MY_TASK2_STACK_BUF_SIZE  1024                   //����2��ջ�����С
CE_STK MY_TASK2_STACK_BUF[MY_TASK2_STACK_BUF_SIZE];     //����2ʹ�ö�ջ����
CeTask myTask2;                                         //��������2�����Զ���ʵ��
void myTask2CallBack(void* pAddPar)                     //ʾ������2������ע�⣺��Creelinks���������񴴽��͵��ý��м򻯴��������������Բ���while(1)ѭ����Ҳ�ܱ�֤���ڵ���
{
    ceDebugOp.printf("Task 2 is running...\n");
    ceSystemOp.delayMs(3000);                           //�ʵ���ʱ���ô�����������������     
}

int main(void)
{
    ceSystemOp.initial();                               //Creelinks������ʼ��
    ceDebugOp.initial(R9Uart);                          //ͨ��Uart�������Debug��Ϣ����λ��
    //TODO:���ڴ˴�����ģ���ʼ�������񴴽��Ȳ���

    myTask1.ID = 0x00001;                               //ָ������1��ID��
    myTask1.taskName = "My Task1";                      //�趨����1�����ƣ�ע�᳤�Ȳ�Ҫ����CeMcu.h�ж����CE_TASK_NAME_LENGTHֵ
    myTask1.callBack = myTask1CallBack;                 //ָ����������1��һ��������Ϊһ������ϵͳ�����ڵ���������
    myTask1.pAddPar = &myTask1;                         //ָ��������1�������ݵ�pAddPar����
    myTask1.isNewThread = 0x01;                         //0x01:������1Ϊһ�����������������������0x00:���д˲���Ϊ0x00��ע�����񣬾���һ��ϵͳ��̨M��������ѭ����
    myTask1.taskPriority = CE_TASK_PRIORITY_M;          //����Creelinksƽ̨������1���ȼ�����5������ΪΪHH��H��M��L��LL��ͬ�����ȼ���������RTOS�����£���ע���������ں�ע�������
    myTask1.taskStackBuf = MY_TASK1_STACK_BUF;          //����ʹ�ö�ջ����
    myTask1.taskStackBufSize = MY_TASK1_STACK_BUF_SIZE; //�����ջ�����С
    ceTaskOp.registerTask(&myTask1);                    //ע�������1
    ceTaskOp.start(&myTask1);                           //��ʼ����

    myTask2.ID = 0x00002;                               //ָ������2��ID��
    myTask2.taskName = "My Task2";                      //�趨����2�����ƣ�ע�᳤�Ȳ�Ҫ����CeMcu.h�ж����CE_TASK_NAME_LENGTHֵ
    myTask2.callBack = myTask2CallBack;                 //ָ����������2��һ��������Ϊһ������ϵͳ�����ڵ���������
    myTask2.pAddPar = &myTask2;                         //ָ�������������ݵ�pAddPar����
    myTask2.isNewThread = 0x01;                         //0x01:������2Ϊһ�����������������������0x00:���д˲���Ϊ0x00��ע�����񣬾���һ��ϵͳ��̨M��������ѭ����
    myTask2.taskPriority = CE_TASK_PRIORITY_HH;         //����Creelinksƽ̨������2���ȼ�����5������ΪΪHH��H��M��L��LL��ͬ�����ȼ���������RTOS�����£���ע���������ں�ע�������
    myTask2.taskStackBuf = MY_TASK2_STACK_BUF;          //����2ʹ�ö�ջ����
    myTask2.taskStackBufSize = MY_TASK2_STACK_BUF_SIZE; //����2��ջ�����С
    ceTaskOp.registerTask(&myTask2);                    //ע�������2
    ceTaskOp.start(&myTask2);                           //��ʼ����

    ceTaskOp.mainTask();                                //��RTOS����ϵͳ��uCOS II�������£�ִ�е��˺����󣬽���������ִ�С�
    return 0;
}
******************************************************************************
*/
