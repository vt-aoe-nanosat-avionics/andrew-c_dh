
#ifndef IS25LP128F_H
#define IS25LP128F_H


// Commands

// Software Reset
#define IS25LP128F_CMD_RESET_ENABLE                   0x66
#define IS25LP128F_CMD_RESET                          0x99

// Read ID
#define IS25LP128F_CMD_FAST_READ                      0x0C
#define IS25LP128F_CMD_QUAD_OUTPUT_FAST_READ          0x6B
#define IS25LP128F_CMD_QUAD_IO_FAST_READ              0xEB

// Erase Operations
#define IS25LP128F_CMD_ERASE_SECTOR                   0x21
#define IS25LP128F_CMD_ERASE_BLOCK                    0x5C

// Write Operations
#define IS25LP128F_CMD_WRITE_ENABLE                   0x06
#define IS25LP128F_CMD_WRITE_DISABLE                  0x04

// Read Register Operations
#define IS25LP128F_CMD_READ_STATUS_REGISTER           0x05
#define IS25LP128F_CMD_READ_READ_PARAMETERS           0x61
#define IS25LP128F_CMD_READ_EXTENDED_READ_PARAMETERS  0x81

// Write Register Operations
#define IS25LP128F_CMD_WRITE_STATUS_REGISTER          0x01
#define IS25LP128F_CMD_WRITE_READ_PARAMETERS          0x63
#define IS25LP128F_CMD_WRITE_EXTENDED_READ_PARAMETERS 0x85

// Qpi Operations
#define IS25LP128F_CMD_ENTER_QPI_MODE                 0x35
#define IS25LP128F_CMD_EXIT_QPI_MODE                  0xF5

// Program Operations
#define IS25LP128F_CMD_PAGE_PROGRAM                   0x12
#define IS25LP128F_CMD_QUAD_PAGE_PROGRAM              0x32


#endif // IS25LP128F_H




