#include "AnalyseData.h"

uint16_t getMidDist(uint16_t dist1,uint16_t dist2){
	if(((dist1-dist2)<=EQUALDIST)||((dist2-dist1)<=EQUALDIST))
		return (dist1+dist2)/2;
	else if(dist1>dist2)
		return dist2;
	else 
		return dist1;	
}