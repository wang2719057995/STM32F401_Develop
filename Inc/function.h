#ifndef __FUNCTION_H
#define __FUNCTION_H

#include "main.h"


float CalculateTemperature(uint16_t ADCValue);
void CalculateResult(uint16_t *ADCValue, uint8_t len, float *ADCResult);
//int KalmanFilter(int inData);
//uint16_t AverageFilter(uint16_t *ValueBuff, uint8_t len);



#endif
