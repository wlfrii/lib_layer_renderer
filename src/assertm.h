#ifndef LIB_LAYER_RENDERER_ASSERTM_H_LF
#define LIB_LAYER_RENDERER_ASSERTM_H_LF


/**
 * Abort with message if a condition is false.
 */
#define ASSERTM(CONDITION, MSG, ...) \
    if(!(CONDITION)) {  \
        printf("***Abort in %s:line:%d: " MSG, \
            __FILE__, __LINE__, ##__VA_ARGS__);\
        std::abort(); \
    }

#endif // LIB_LAYER_RENDERER_ASSERTM_H_LF
