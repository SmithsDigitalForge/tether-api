#ifndef _TETHER_LOGGER_H_
#define _TETHER_LOGGER_H_

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else  // ! __cplusplus
#define EXTERN_C
#endif  // __cplusplus

#include <unistd.h>

extern void (*tether_info_logger)(const char *format, ...);
extern void (*tether_error_logger)(const char *file, int line, const char *format, ...);

#define LOG_INFO(fmt, ...)                                        \
    do {                                                          \
        tether_info_logger(fmt, ##__VA_ARGS__); \
    } while (0);

#define LOG_ERROR(fmt, ...)                                        \
    do {                                                           \
        tether_error_logger(__FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    } while (0);

#endif  // _TETHER_LOGGER_H_
