#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <stdio.h>
#if defined Linux
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#elif defined _WINDOWS
#include <Windows.h>
#endif
#include "mrapi.h"
#ifndef LOG_LEVEL
#define LOG_LEVEL 1
#endif
#ifndef LOG_APPEND
#define LOG_APPEND 1
#endif

#define PLOG(level, ...) \
    {						\
        if(logp==NULL) {		\
                loggerInit(&logp);	\
        }                    \
        loggerTime(sstime);  \
        fprintf(logp,"%s%s", sstime, level); \
        fprintf(logp,__VA_ARGS__);		\
        fprintf(logp,"\n");	\
        fflush(logp);			\
    }						\

#define RLOG(...)  \
    {						\
        if (logp == NULL) {  \
            loggerInit(&logp);	\
        }                    \
        fprintf(logp, __VA_ARGS__);		\
        fflush(logp);			\
    }						\

#define LOG_TIME()   \
    {						\
        if (logp == NULL) {  \
                loggerInit(&logp);	\
        }                    \
        loggerTime(sstime);  \
        fprintf(logp, "%s", sstime); \
        fflush(logp);			\
    }\

#if (LOG_LEVEL==4)
#define DLOG( ...) PLOG(" | DEBUG | ",__VA_ARGS__)
#define ILOG( ...) PLOG(" | INFO  | ",__VA_ARGS__)
#define WLOG( ...) PLOG(" | WARN  | ",__VA_ARGS__)
#define ELOG( ...) PLOG(" | ERROR | ",__VA_ARGS__)
#elif (LOG_LEVEL==3)
#define DLOG( ...)
#define ILOG( ...) PLOG(" | INFO  | ",__VA_ARGS__)
#define WLOG( ...) PLOG(" | WARN  | ",__VA_ARGS__)
#define ELOG( ...) PLOG(" | ERROR | ",__VA_ARGS__)
#elif (LOG_LEVEL==2)
#define DLOG( ...)
#define ILOG( ...)
#define WLOG( ...) PLOG(" | WARN  | ",__VA_ARGS__)
#define ELOG( ...) PLOG(" | ERROR | ",__VA_ARGS__)
#elif (LOG_LEVEL==1)
#define DLOG( ...)
#define ILOG( ...)
#define WLOG( ...)
#define ELOG( ...) PLOG(" | ERROR | ",__VA_ARGS__)
#elif (LOG_LEVEL==0)
#define DLOG( ...)
#define ILOG( ...)
#define WLOG( ...)
#define ELOG( ...) 
#endif

FILE *logp;
char sstime[256];

void loggerInit(FILE **fp);
void loggerTime(char*);

#endif /*_LOGGER_H*/
