#ifndef LOGGER_H_LF
#define LOGGER_H_LF



#define WLF_VERBOSE(fmt, ...) \
    printf("[%s] " fmt, __func__, ##__VA_ARGS__)


#define WLF_LOG(fmt, ...) \
    printf("[%s][%s][%d] " fmt, __FILE__, __func__, __LINE__, ##__VA_ARGS__)



#endif // LOGGER_H_LF
