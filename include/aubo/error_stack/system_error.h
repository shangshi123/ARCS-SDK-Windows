/** @file  system_error.h
 *  @brief 系统错误码
 */
#ifndef AUBO_SDK_SYSTEM_ERROR_H
#define AUBO_SDK_SYSTEM_ERROR_H

#define SYSTEM_ERRORS                                                         \
    _D(DEBUG, 0, "Debug message " _PH1_, "suggest...")                        \
    _D(POPUP, 1, "Popup title: " _PH1_ ", msg: " _PH2_ ", mode: " _PH3_,      \
       "suggest...")                                                          \
    _D(POPUP_DISMISS, 2, _PH1_, "suggest...")                                 \
    _D(SYSTEM_HALT, 3, _PH1_, "suggest...")                                   \
    _D(INV_ARGUMENTS, 4, "Invalid arguments.", "suggest...")                  \
    _D(USER_NOTIFY, 5, _PH1_, "suggest...")                                   \
    _D(MODBUS_SIGNAL_CREATED, 10, "Modbus signal " _PH1_ " created.",         \
       "suggest...")                                                          \
    _D(MODBUS_SIGNAL_REMOVED, 11, "Modbus signal " _PH1_ " removed.",         \
       "suggest...")                                                          \
    _D(MODBUS_SIGNAL_VALUE_CHANGED, 12,                                       \
       "Modbus signal " _PH1_ " value changed to " _PH2_, "suggest...")       \
    _D(RUNTIME_CONTEXT, 13,                                                   \
       "tid: " _PH1_ " lineno: " _PH2_ " index: " _PH3_ " comment: " _PH4_,   \
       "suggest...")                                                          \
    _D(INTERP_CONTEXT, 14,                                                    \
       "tid: " _PH1_ " lineno: " _PH2_ " index: " _PH3_ " comment: " _PH4_,   \
       "suggest...")                                                          \
    _D(MODBUS_SLAVE_BIT, 20, "Modbus slave address: " _PH1_ " value " _PH2_,  \
       "suggest...")                                                          \
    _D(MODBUS_SLAVE_REG, 21, "Modbus slave address: " _PH1_ " value " _PH2_,  \
       "suggest...")                                                          \
    _D(PNIO_SLAVE_SLOT_VALUE, 30,                                             \
       "PNIO slot: " _PH1_ " subslot " _PH2_ " index " _PH3_ " value " _PH4_, \
       "suggest...")                                                          \
    _D(ICM_SERVER_STATUS, 40, " ICM server status changed to " _PH1_,         \
       "suggest...")                                                          \
    _D(EIP_X, 50, "ICM slot: " _PH1_ " index " _PH2_ " subindex " _PH3_,      \
       "suggest...")

#endif // AUBO_SDK_SYSTEM_ERROR_H
