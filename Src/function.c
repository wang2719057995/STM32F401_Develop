#include "function.h"



float CalculateTemperature(uint16_t ADCValue)
{
	float Temperature;
	if(ADCValue<0 || ADCValue>4095)
		return 99;
	//uint16_t TS_CAL1 = *(__IO uint16_t *)(0x1FFF75A8);
  //uint16_t TS_CAL2 = *(__IO uint16_t *)(0x1FFF75CA);
	//tempvalue = (110 - 30) * (ADCValue*1.1 - TS_CAL1)/ (TS_CAL2 - TS_CAL1) + 30;
	Temperature=(float)ADCValue*(3.3/4096);  //计算传感器的电压值给到温度
	Temperature=(Temperature-0.76)/0.0025+25;  //通过公式计算出来的温度值
	return Temperature;
}

void CalculateResult(uint16_t *ADCValue, uint8_t len,  float *ADCResult)
{
	//float ADCResult[3];
	float alpha = 1.0054875;
	//for version 2
	ADCResult[0] = (ADCValue[5]*3.3/4096.0) / 20.0 / 0.02;
	//for version 1
	//ADCResult[0] = (ADCValue[1]*3.3/4096.0) / 20.0 / 0.02;
	//ADCResult[0] = (ADCValue[1]*3.3/4096.0) / 20.0 / 0.02;
	ADCResult[1] = (ADCValue[2]*3.3/4096.0) / (4.0/(78.0+4.0)) * 1.0112;
	ADCResult[2] = (ADCValue[3]*3.3/4096.0) / (4.0/(16.0+4.0)) * 1.021;
	ADCResult[3] = (ADCValue[4]*3.3/4096.0) / 20.0 / 0.02;
	
}




