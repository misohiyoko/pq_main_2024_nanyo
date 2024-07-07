//
// Created by 児玉源太郎 on 2024/07/04.
//

#include "gnss.h"

#include "stm32l4xx_hal.h"

uint8_t nmea_buffer[MINMEA_MAX_SENTENCE_LENGTH];
size_t nmea_buffer_pos = 0;
gnss_data_t gnss_data;
int feed_nmea(uint8_t code){
    if(code == '\n' || code == '\r'){
        nmea_buffer[nmea_buffer_pos] = '\0';
        nmea_buffer_pos = 0;

        switch (minmea_sentence_id(nmea_buffer, false))
        {
            case MINMEA_INVALID:
                return MINMEA_INVALID;
            case MINMEA_UNKNOWN:
                return MINMEA_INVALID;
            case MINMEA_SENTENCE_GBS:
                break;
            case MINMEA_SENTENCE_GGA:
                if(minmea_parse_gga(&gnss_data.gga, nmea_buffer)){
                    gnss_data.gga_time = HAL_GetTick();

                }
                return 0;
            case MINMEA_SENTENCE_GLL:
                break;
            case MINMEA_SENTENCE_GSA:
                if(minmea_parse_gsa(&gnss_data.gsa, nmea_buffer)){
                    gnss_data.gsa_time = HAL_GetTick();

                }
                return 0;
            case MINMEA_SENTENCE_GST:
                break;
            case MINMEA_SENTENCE_GSV:
                break;
            case MINMEA_SENTENCE_RMC:
                break;
            case MINMEA_SENTENCE_VTG:
                break;
            case MINMEA_SENTENCE_ZDA:
                struct minmea_sentence_zda zda;
                if(minmea_parse_zda(&zda, nmea_buffer)){
                    gnss_data.recent_timestamp = HAL_GetTick();
                    minmea_getdatetime(&gnss_data.recent_time, &zda.date, &zda.time);
                }
                return 0;
        }
        return -2;
    }else{
        nmea_buffer[nmea_buffer_pos++] = code;
        if(nmea_buffer_pos >= MINMEA_MAX_SENTENCE_LENGTH){
            nmea_buffer_pos = 0;
            return MINMEA_INVALID;
        }
    }
}