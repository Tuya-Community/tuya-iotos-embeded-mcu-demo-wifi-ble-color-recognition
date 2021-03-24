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
	IIC_SDA_OUT;//sda线输出
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_RESET;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1; 
	delay_us(5);
	IIC_SDA_SET;//IIC_SDA=1;//发送I2C总线结束信号				   	
}
void IIC_Ack(void)//产生ACK应答
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_RESET;//IIC_SDA=0;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}   
void IIC_NAck(void)//不产生ACK应答	
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_SET;//IIC_SDA=1;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}	
uint8_t IIC_Wait_Ack(void)//等待应答信号到来:1,接收应答失败;0,接收应答成功
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA设置为输入     
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);	 
	while(IIC_SDA_State)//检测SDA是否仍为高电平
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
void IIC_Send_Byte(uint8_t txd)//IIC发送一个字节; 先发送高位
{                        
	uint8_t t;   
	IIC_SDA_OUT; 	    
	IIC_SCL_RESET;//IIC_SCL=0;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
			//IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_SET;//IIC_SDA=1;
		else
			IIC_SDA_RESET;//IIC_SDA=0;
		
		txd<<=1; 	  
		delay_us(5);   //对TEA5767这三个延时都是必须的
		IIC_SCL_SET;//IIC_SCL=1;
		delay_us(5); 
		IIC_SCL_RESET;//IIC_SCL=0;	
		delay_us(5);
	}	 
} 
uint8_t IIC_Read_Byte(unsigned char ack)//读一个字节，可加是否应答位,1加ack，0不加ack 从高位开始读
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA设置为输入
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
			IIC_Ack(); //发送ACK
		else
			IIC_NAck();//发送nACK   
		return receive;
}

uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data)//直接写一个字节
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //发送写命令
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//发送地址	  
	ret |= IIC_Wait_Ack();		
	IIC_Send_Byte(data);     //发送字节							   
	ret |= IIC_Wait_Ack(); 

	IIC_Stop();
	delay_us(10);
	return ret;
}
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)//读字节
{  	    																 
	uint8_t ret=0;
	
	IIC_Start();  
	IIC_Send_Byte(DrvAddr);	   //发送写命令
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);//发送地址	    
	ret |= IIC_Wait_Ack();	    
	
	IIC_Start();
	IIC_Send_Byte(DrvAddr+1);           //进入接收模式			   
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
	IIC_Stop();//产生一个停止条件	
	return ret;	
}
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)//可一次写多个字节
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //发送写命令
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//发送地址	  
	ret |= IIC_Wait_Ack();		

	while(NumToWrite--)
	{ 										  		   
		IIC_Send_Byte(*pBuffer);     //发送字节							   
		ret |= IIC_Wait_Ack(); 
		pBuffer++;
	}
	IIC_Stop();
	delay_us(10);
	return ret;
}
#endif
/******************************************************颜色识别******************************************************/
uint8_t F_TASK_TCS34725=0;
#if USING_TCS34725
COLOR_RGBC rgb;
COLOR_RGBC color_ratio;
/*******************************************************************************
 * @brief TCS34725设置积分时间
 *
 * @return None
*******************************************************************************/
void TCS34725_SetIntegrationTime(uint8_t time)
{
	IIC_WriteMulByte(TCS34725_ADDRESS,TCS34725_ATIME| TCS34725_COMMAND_BIT, &time, 1);
}
/*******************************************************************************
 * @brief TCS34725设置增益
 *
 * @return None
*******************************************************************************/
void TCS34725_SetGain(uint8_t gain)
{
	IIC_WriteMulByte(TCS34725_ADDRESS,TCS34725_CONTROL| TCS34725_COMMAND_BIT, &gain, 1);
}
/*******************************************************************************
 * @brief TCS34725使能
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
 * @brief TCS34725失能
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
 * @brief TCS34725初始化
 *
 * @return ID - ID寄存器中的值
*******************************************************************************/
void TCS34725_Init(void)
{
	uint8_t id=0;
	 
	IIC_ReadMulByte(TCS34725_ADDRESS,TCS34725_ID| TCS34725_COMMAND_BIT, &id, 1);  //TCS34725 的 ID 是 0x44 可以根据这个来判断是否成功连接
	if(id==0x44)
		{
			TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
			TCS34725_SetGain(TCS34725_GAIN_1X);
			TCS34725_Enable();
		}
}
/*******************************************************************************
 * @brief TCS34725获取单个通道数据
 *
 * @return data - 该通道的转换值
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
 * @brief TCS34725获取各个通道数据
 *
 * @return 1 - 转换完成，数据可用
 *   	   0 - 转换未完成，数据不可用
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
/******************************************************颜色识别******************************************************/
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
