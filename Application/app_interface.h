/*
 * main_cppwrapper.h
 *
 *  Created on: Aug 12, 2024
 *      Author: 42077
 */

#ifndef APP_INTERFACE_H_
#define APP_INTERFACE_H_

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC void noreturn_app(void);
EXTERNC void uart_cb(void * handle);
EXTERNC void adc_cb(void * handle);

#undef EXTERNC
// ...

#endif /* APP_INTERFACE_H_ */
