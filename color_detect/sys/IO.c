#include "IO.h"
#include "delay.h"
#include "TIM.h"
#include "math.h"
#include "stdlib.h"
#if 1//IIC
void IIC_Init(void)
{
	IIC_SCL_OUT;
	IIC_SDA_OUT;
}
void IIC_Start(void)//Start
{
	IIC_SDA_OUT;     
	IIC_SDA_SET;	  
	IIC_SCL_SET;
	delay_us(5);
	IIC_SDA_RESET;//IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}
void IIC_Stop(void)//Stop
{
	IIC_SDA_OUT;//sda�����
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_RESET;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1; 
	delay_us(5);
	IIC_SDA_SET;//IIC_SDA=1;//����I2C���߽����ź�				   	
}
void IIC_Ack(void)//����ACKӦ��
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_RESET;//IIC_SDA=0;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}   
void IIC_NAck(void)//������ACKӦ��	
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_SET;//IIC_SDA=1;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}	
uint8_t IIC_Wait_Ack(void)//�ȴ�Ӧ���źŵ���:1,����Ӧ��ʧ��;0,����Ӧ��ɹ�
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA����Ϊ����     
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);	 
	while(IIC_SDA_State)//���SDA�Ƿ���Ϊ�ߵ�ƽ
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_RESET;//IIC_SCL=0;
	return 0;  
} 
void IIC_Send_Byte(uint8_t txd)//IIC����һ���ֽ�; �ȷ��͸�λ
{                        
	uint8_t t;   
	IIC_SDA_OUT; 	    
	IIC_SCL_RESET;//IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
			//IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_SET;//IIC_SDA=1;
		else
			IIC_SDA_RESET;//IIC_SDA=0;
		
		txd<<=1; 	  
		delay_us(5);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_SET;//IIC_SCL=1;
		delay_us(5); 
		IIC_SCL_RESET;//IIC_SCL=0;	
		delay_us(5);
	}	 
} 
uint8_t IIC_Read_Byte(unsigned char ack)//��һ���ֽڣ��ɼ��Ƿ�Ӧ��λ,1��ack��0����ack �Ӹ�λ��ʼ��
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		IIC_SCL_RESET;// IIC_SCL=0; 
		delay_us(5);
		IIC_SCL_SET;//IIC_SCL=1;
		receive<<=1;
		if(IIC_SDA_State)
			receive++;   
		delay_us(5); 
	}					 
		if (ack)
			IIC_Ack(); //����ACK
		else
			IIC_NAck();//����nACK   
		return receive;
}

uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data)//ֱ��дһ���ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		
	IIC_Send_Byte(data);     //�����ֽ�							   
	ret |= IIC_Wait_Ack(); 

	IIC_Stop();
	delay_us(10);
	return ret;
}
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)//���ֽ�
{  	    																 
	uint8_t ret=0;
	
	IIC_Start();  
	IIC_Send_Byte(DrvAddr);	   //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);//���͵�ַ	    
	ret |= IIC_Wait_Ack();	    
	
	IIC_Start();
	IIC_Send_Byte(DrvAddr+1);           //�������ģʽ			   
	ret |= IIC_Wait_Ack();
	while(NumToRead)
	{
		if(NumToRead==1)
		{
			*pBuffer=IIC_Read_Byte(0);	
		}
		else
		{
			*pBuffer=IIC_Read_Byte(1);
		}
		pBuffer++;
		NumToRead--;
	}
	IIC_Stop();//����һ��ֹͣ����	
	return ret;	
}
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)//��һ��д����ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		

	while(NumToWrite--)
	{ 										  		   
		IIC_Send_Byte(*pBuffer);     //�����ֽ�							   
		ret |= IIC_Wait_Ack(); 
		pBuffer++;
	}
	IIC_Stop();
	delay_us(10);
	return ret;
}
#endif
/******************************************************��ɫʶ��******************************************************/
uint8_t F_TASK_TCS34725=0;
#if USING_TCS34725
COLOR_RGBC rgb;
COLOR_RGBC color_ratio;
/*******************************************************************************
 * @brief TCS34725���û���ʱ��
 *
 * @return None
*******************************************************************************/
void TCS34725_SetIntegrationTime(uint8_t time)
{
	IIC_WriteMulByte(TCS34725_ADDRESS,TCS34725_ATIME| TCS34725_COMMAND_BIT, &time, 1);
}
/*******************************************************************************
 * @brief TCS34725��������
 *
 * @return None
*******************************************************************************/
void TCS34725_SetGain(uint8_t gain)
{
	IIC_WriteMulByte(TCS34725_ADDRESS,TCS34725_CONTROL| TCS34725_COMMAND_BIT, &gain, 1);
}
/*******************************************************************************
 * @brief TCS34725ʹ��
 *
 * @return None
*******************************************************************************/
void TCS34725_Enable(void)
{
	uint8_t cmd = TCS34725_ENABLE_PON;
	
	IIC_ReadMulByte(TCS34725_ADDRESS,TCS34725_ENABLE| TCS34725_COMMAND_BIT, &cmd, 1);
	cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
	IIC_WriteMulByte(TCS34725_ADDRESS,TCS34725_ENABLE| TCS34725_COMMAND_BIT, &cmd, 1);
}
/*******************************************************************************
 * @brief TCS34725ʧ��
 *
 * @return None
*******************************************************************************/
void TCS34725_Disable(void)
{
	uint8_t cmd = 0;
	
	IIC_ReadMulByte(TCS34725_ADDRESS,TCS34725_ENABLE| TCS34725_COMMAND_BIT, &cmd, 1);
	cmd = cmd & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
	IIC_WriteMulByte(TCS34725_ADDRESS,TCS34725_ENABLE| TCS34725_COMMAND_BIT, &cmd, 1);
}
/*******************************************************************************
 * @brief TCS34725��ʼ��
 *
 * @return ID - ID�Ĵ����е�ֵ
*******************************************************************************/
void TCS34725_Init(void)
{
	uint8_t id=0;
	 
	IIC_ReadMulByte(TCS34725_ADDRESS,TCS34725_ID| TCS34725_COMMAND_BIT, &id, 1);  //TCS34725 �� ID �� 0x44 ���Ը���������ж��Ƿ�ɹ�����
	if(id==0x44)
		{
			TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
			TCS34725_SetGain(TCS34725_GAIN_1X);
			TCS34725_Enable();
		}
}
/*******************************************************************************
 * @brief TCS34725��ȡ����ͨ������
 *
 * @return data - ��ͨ����ת��ֵ
*******************************************************************************/
uint16_t TCS34725_GetChannelData(uint8_t reg)
{
	uint8_t tmp[2] = {0,0};
	uint16_t data;
	
	IIC_ReadMulByte(TCS34725_ADDRESS,reg| TCS34725_COMMAND_BIT, tmp, 2);
	data = (tmp[1] << 8) | tmp[0];
	
	return data;
}
/*******************************************************************************
 * @brief TCS34725��ȡ����ͨ������
 *
 * @return 1 - ת����ɣ����ݿ���
 *   	   0 - ת��δ��ɣ����ݲ�����
*******************************************************************************/
uint8_t TCS34725_GetRawData(COLOR_RGBC *rgbc)
{
	uint8_t status = TCS34725_STATUS_AVALID;
	
	IIC_ReadMulByte(TCS34725_ADDRESS,TCS34725_STATUS| TCS34725_COMMAND_BIT, &status, 1);
	
	if(status & TCS34725_STATUS_AVALID)
	{
		rgbc->c = TCS34725_GetChannelData(TCS34725_CDATAL);	
		rgbc->r = TCS34725_GetChannelData(TCS34725_RDATAL);	
		rgbc->g = TCS34725_GetChannelData(TCS34725_GDATAL);	
		rgbc->b = TCS34725_GetChannelData(TCS34725_BDATAL);
		return 1;
	}
	return 0;
}
void RAWtoRGB(COLOR_RGBC *rgbc, COLOR_RGBC *rgb)
{
	 rgb->r = rgbc->r*255/rgbc->c;  
	 rgb->g = rgbc->g*255/rgbc->c;
	 rgb->b = rgbc->b*255/rgbc->c;
}
void TASK_TCS34725(void)
{
	if(TCS34725_GetRawData(&rgb))  
	{
		RAWtoRGB(&rgb,&color_ratio);
		mcu_dp_value_update(DPID_RED,color_ratio.r);
		mcu_dp_value_update(DPID_GREEN,color_ratio.g);
		mcu_dp_value_update(DPID_BLUE,color_ratio.b);
	}
}
#endif
/******************************************************��ɫʶ��******************************************************/
void Modules_Init(void)
{
	IIC_Init();
	TCS34725_Init();
}
void SwitchIO_Init(void)
{
	LED_4_OUT;	
}
void IO_Init(void)
{
	Modules_Init();
	SwitchIO_Init();
}
