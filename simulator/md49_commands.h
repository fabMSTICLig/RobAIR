#ifndef MD49SIM_COMMANDS_H
#define MD49SIM_COMMANDS_H

#define MD49_VERSION               1

#define MD49_SYNC                  0x00

#define MD49_CMD_GET_SPEED_1       0x21
#define MD49_CMD_GET_SPEED_2       0x22
#define MD49_CMD_GET_ENCODER_1     0x23
#define MD49_CMD_GET_ENCODER_2     0x24
#define MD49_CMD_GET_ENCODERS      0x25
#define MD49_CMD_GET_VOLTS         0x26
#define MD49_CMD_GET_CURRENT_1     0x27
#define MD49_CMD_GET_CURRENT_2     0x28
#define MD49_CMD_GET_VERSION       0x29
#define MD49_CMD_GET_ACCELERATION  0x2a
#define MD49_CMD_GET_MODE          0x2b
#define MD49_CMD_GET_VI            0x2c
#define MD49_CMD_GET_ERROR         0x2d
#define MD49_CMD_SET_SPEED_1       0x31
#define MD49_CMD_SET_SPEED_2       0x32
#define MD49_CMD_SET_ACCELERATION  0x33
#define MD49_CMD_SET_MODE          0x34
#define MD49_CMD_RESET_ENCODERS    0x35
#define MD49_CMD_DISABLE_REGULATOR 0x36
#define MD49_CMD_ENABLE_REGULATOR  0x37
#define MD49_CMD_DISABLE_TIMEOUT   0x38
#define MD49_CMD_ENABLE_TIMEOUT    0x39

#endif
