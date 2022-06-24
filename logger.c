#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>

#include "logger.h"
#include "ice9.h"

static pthread_mutex_t s_log_lock = PTHREAD_MUTEX_INITIALIZER;

static void tether_log_info(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    buf[sizeof(buf)-1] = '\0';

    pthread_mutex_lock(&s_log_lock);
    printf("%s", buf);
    pthread_mutex_unlock(&s_log_lock);
}

static void tether_log_error(const char *file, int line, const char *format, ...) {
    char buf[256];
    const int nwritten = snprintf(buf, sizeof(buf), "%s:%d ", file, line);
    if (nwritten < sizeof(buf)-1) {
        va_list args;
        va_start(args, format);
        vfprintf(stderr, format, args);
        vsnprintf(buf, sizeof(buf)-nwritten, format, args);
        va_end(args);
    }
    buf[sizeof(buf)-1] = '\0';

    pthread_mutex_lock(&s_log_lock);
    fprintf(stderr, "%s", buf);
    pthread_mutex_unlock(&s_log_lock);
}

void (*tether_info_logger)(const char *format, ...) = tether_log_info;
void (*tether_error_logger)(const char *file, int line, const char *format, ...) = tether_log_error;

void tether_set_info_logger(void (*log_info)(const char *format, ...)) {
    pthread_mutex_lock(&s_log_lock);
    tether_info_logger = log_info;
    pthread_mutex_unlock(&s_log_lock);
    ice9_set_info_logger(log_info);
}

void tether_set_error_logger(void (*log_error)(const char *file, int line, const char *format, ...)) {
    pthread_mutex_lock(&s_log_lock);
    tether_error_logger = log_error;
    pthread_mutex_unlock(&s_log_lock);
    ice9_set_error_logger(log_error);
}
