#include "logger.h"

void loggerInit(FILE **fp)
{
#if  LOG_APPEND == 0 
	SYSTEMTIME sys;
	char path[256];
	GetLocalTime(&sys);
	sprintf(path,"./%4d%02d%02d-%02d%02d%02d.log", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond);
	*fp = fopen(path, "a");
	if (*fp == NULL)
		printf("fopen logfile");
#elif LOG_APPEND == 1
	*fp = stdout;
#elif LOG_APPEND == 2
	* fp = stderr;
#endif
}

 void loggerTime(char* stime) {
	LARGE_INTEGER m_nFreq;
	LARGE_INTEGER m_nTime;
	QueryPerformanceFrequency(&m_nFreq); // 获取时钟周期 
	QueryPerformanceCounter(&m_nTime);//获取当前时间 
	sprintf(stime, "%.4f", ((double)m_nTime.QuadPart / m_nFreq.QuadPart));
}