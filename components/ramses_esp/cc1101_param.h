/*********************************************************
*
* cc1101_param.h
* Parameter management for cc1101
*/
#ifndef _CC1101_PARAM_H_
#define _CC1101_PARAM_H_

#include <stdint.h>

#include "cc_const.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t cc_cfg_get(uint8_t param, uint8_t *buff, uint8_t nParam);
extern uint8_t cc_pa_get(uint8_t *paTable);

#ifdef __cplusplus
}
#endif

#endif // _CC1101_PARAM_H_