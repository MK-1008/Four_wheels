#ifndef __KEY_CTRL_H__
#define __KEY_CTRL_H__
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>

enum ControlMode { MANUAL, AUTOMATIC };
extern enum ControlMode controlMode;

void read_button_state(void);


#ifdef __cplusplus
}
#endif

#endif /* __KEY_CTRL_H__ */
