#include "stdint.h"




struct ArrStr
{
	uint16_t InputValue;
  float OutputValue;
};

struct ArrStrF
{
	float InputValue;
  float OutputValue;
};

struct ArrStrF TempCorrectionDependence[] = 
{
	70, 10.0 / 23.0,
	50, 11.0/ 23.0,
	34, 12.0/ 23.0,
	26, 16.0/ 23.0,
	18, 16.0/ 23.0,
	10, 18.0/ 23.0,
	4, 63.0/ 23.0,
	0, 63.0/ 23.0
};

struct ArrStr TempDependence[] = 
{
4219,	-4,
4022,	0,
3737,	5,
3433,	10,
3119,	15,
2803,	20,
2491,	25,
2219,	30,
1918,	35,
1665,	40,
1438,	45,
1237,	50,
913,	60,
673,	70,
496,	80,
370,	90,
276,	100
};



struct ArrStr WQ_20[] = 
{
	3900, 0,
	3400, 1,
	3000, 1.5,
	1920, 5.3,
	//1580, 8,
	1280, 11,
	1110, 15,
	940 , 20.7,
	850 , 25,
	740 , 32,
	620, 48,
	480, 100,
	0, 1000
};

/* старое struct ArrStr WQ_20[] = 
{
	4100,	0,
	3100, 1,
	1950, 5.2,
	1550, 10,
	1265, 12,
	1050, 20,
	450,  500,
	0,    1000
};
*/

double GetDependenceValueF(double Val , struct ArrStrF * array,	uint8_t size)
{
  if (Val > array[0].InputValue) return 0;
	for (int i = 1; i < size; i++)
	{
		if (Val > array[i].InputValue)
  		return	(array[i - 1].OutputValue + (array[i].OutputValue - array[i - 1].OutputValue) * (Val - array[i - 1].InputValue) / (array[i].InputValue - array[i-1].InputValue));
		}
	return 0.0;
};


double GetDependenceValue(uint16_t adcVal , struct ArrStr * array,	uint8_t size)
{
  if (adcVal > array[0].InputValue) return 0;
	for (int i = 1; i < size; i++)
	{
		if (adcVal > array[i].InputValue)
  		return	(array[i - 1].OutputValue + (array[i].OutputValue - array[i - 1].OutputValue) * (adcVal - array[i - 1].InputValue) / (array[i].InputValue - array[i-1].InputValue));
		}
	return 0.0;
};