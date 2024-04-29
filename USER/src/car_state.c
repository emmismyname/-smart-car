#include "headfile.h"
#include "car_state.h"
//����һ��״̬�Ľṹ��


uint8 SWITCH1_CURRENTSTATE=0;
uint8 SWITCH2_CURRENTSTATE=0;
uint8 SWITCH3_CURRENTSTATE=0;
uint8 SWITCH4_CURRENTSTATE=0;

uint8 SWITCH1_FLAG=0;
uint8 SWITCH2_FLAG=0;
uint8 SWITCH3_FLAG=0;
uint8 SWITCH4_FLAG=0;


car_states jiangxing_No_1;//���巢��״̬

//-------------------------------------------------------------------------------------------------------------------
//   @brief      pid����������ʼ��
//   @param      pid				pid����
//   @param      error			pid�������
//   @return     PID������
//-------------------------------------------------------------------------------------------------------------------
int car_init(car_states *name)
{
  name->stop_flag=1;
  name->start_flag=0;
  name->start_mode=0;
  name->left_target_speed=0;
  name->right_target_speed=0;
  name->car_speed=0;
  name->uart_mode=1;
  name->trace_flag=0;
  name->motor_flag=0;
  name->running_mode=0;
  return 1;

}

//-------------------------------------------------------------------------------------------------------------------
//   @brief      
//   @param      
//   @param      
//   @return     
//-------------------------------------------------------------------------------------------------------------------	
void parameter_init()
{
  jiangxing_No_1.uart_mode=1;
}




//-------------------------------------------------------------------------------------------------------------------
//   @brief      
//   @param      
//   @param      
//   @return     
//-------------------------------------------------------------------------------------------------------------------	
void get_switch()
{
  SWITCH1_CURRENTSTATE=switch_1;
  SWITCH2_CURRENTSTATE=switch_2;
  SWITCH3_CURRENTSTATE=switch_3;
  SWITCH4_CURRENTSTATE=switch_4;
  // if((SWITCH1_CURRENTSTATE!=SWITCH1_FLAG)||(SWITCH2_CURRENTSTATE!=SWITCH2_FLAG)||(SWITCH3_CURRENTSTATE!=SWITCH3_FLAG)||(SWITCH4_CURRENTSTATE!=SWITCH4_FLAG))
  // {
   
  // }
  if(SWITCH1_CURRENTSTATE==0&&SWITCH2_CURRENTSTATE==1)
  { 
    jiangxing_No_1.start_mode=1;//��ʱ��
     SWITCH1_FLAG=1;
  }
  else if(SWITCH2_CURRENTSTATE==0&&SWITCH1_CURRENTSTATE==1)
  {
    SWITCH2_FLAG=1;
   jiangxing_No_1.start_mode=2;//������
  }
  else
  {
    SWITCH1_FLAG=0;
    SWITCH2_FLAG=0;
    jiangxing_No_1.start_mode=0;
  }
  

  
  if(SWITCH3_CURRENTSTATE==0)//����3=on
  { 
    SWITCH3_FLAG=1;
  }
  if(SWITCH3_CURRENTSTATE==1)//����3
  { 
    SWITCH3_FLAG=0;
  }



  if(SWITCH4_CURRENTSTATE==0)//����4
  { 
    SWITCH4_FLAG=1;
    para_change_flag=1;//����spi���ݲ��޸�PID��������ģʽ

  }
  else if(SWITCH4_CURRENTSTATE==1)//����4
  { 
    SWITCH4_FLAG=0;
    para_change_flag=0;//�رս���spi���ݹرյ���ģʽ
  }



}


//-------------------------------------------------------------------------------------------------------------------
//   @brief      
//   @param      
//   @param      
//   @return     
//-------------------------------------------------------------------------------------------------------------------	
void switch_init()
{
//gpio_mode(switch_1,GPI_IMPEDANCE);
//gpio_mode(switch_2,GPI_IMPEDANCE);
//gpio_mode(switch_3,GPI_IMPEDANCE);
//gpio_mode(switch_4,GPI_IMPEDANCE);
	
gpio_mode(switch_1,GPI_OD);
gpio_mode(switch_2,GPI_OD);
gpio_mode(switch_3,GPI_OD);
gpio_mode(switch_4,GPI_OD);
}