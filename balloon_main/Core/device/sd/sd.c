//
// Created by genta on 2024/06/07.
//

#include "sd.h"


extern char SDPath[4];

                                        /* FatFs function common result code */
FATFS sd_fat_fs_;  /* File system object for SD card logical drive */
FIL my_file_;     /* File object */
char *sd_path_ = SDPath; /* SD card logical drive path */

uint8_t wtext[] = "BOOTED"; /* File write buffer */
char init_file_name[] = "BOOTLOG.TXT\0";



int init_sd(){
    FRESULT f_res;
    FATFS_TRY(f_mount(&sd_fat_fs_, sd_path_, 0))
    FATFS_TRY(f_open(&my_file_, "LOG.TXT\0", FA_OPEN_ALWAYS | FA_WRITE))
    f_lseek(&my_file_, f_size(&my_file_));

    for(int i = 0; i < 5; i++){
        UINT byteswritten;
        f_res = f_write(&my_file_, wtext, sizeof(wtext), &byteswritten);
        if((byteswritten != 0) && (f_res == FR_OK))
        {
            HAL_Delay(50);
            break;
        }
        HAL_Delay(50);
        printf("cannot write");
    }
    f_close(&my_file_);
}