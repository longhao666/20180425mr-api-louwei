#include "logger.h"

FILE *logp;
char sstime[256];

void loggerInit(FILE **fp)
{
#if  LOG_APPEND == 0
	char path[256];
#if defined _WINDOWS
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	sprintf(path, "./%4d%02d%02d-%02d%02d%02d.log", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond);
#elif defined Linux
	struct tm *local;
	time_t t;
	t = time(NULL);
	local = localtime(&t);
	sprintf(path, "./%4d%02d%02d-%02d%02d%02d.log", local->tm_year+1900, local->tm_mon+1, local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);
#endif
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
#if defined _WINDOWS
    LARGE_INTEGER m_nFreq;
	LARGE_INTEGER m_nTime;
	QueryPerformanceFrequency(&m_nFreq); // 获取时钟周期 
    QueryPerformanceCounter(&m_nTime);   //获取当前时间
	sprintf(stime, "%.4f", ((double)m_nTime.QuadPart / m_nFreq.QuadPart));
#elif defined Linux
	struct timeval tv;
	gettimeofday(&tv, NULL);
	sprintf(stime, "%.4f", ((double)tv.tv_sec + (double)tv.tv_usec/1000000));
#endif
}
