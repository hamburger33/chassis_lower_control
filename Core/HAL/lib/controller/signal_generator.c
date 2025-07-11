#include "signal_generator.h"

float Signal_Generator(signal_generator* obj) {
    obj->Signal_Cnt++;
    if (obj->mode == 1) {
        if (obj->Signal_Cnt <= 2000) {
            return 1;
        } else if (obj->Signal_Cnt > 2000 && obj->Signal_Cnt <= 4000) {
            return -1;
        } else if (obj->Signal_Cnt > 4000 && obj->Signal_Cnt <= 6000) {
            return 2;
        } else if (obj->Signal_Cnt > 6000 && obj->Signal_Cnt <= 8000) {
            return -2;
        } else if (obj->Signal_Cnt > 8000 && obj->Signal_Cnt <= 10000) {
            return 3;
        } else if (obj->Signal_Cnt > 10000 && obj->Signal_Cnt <= 12000) {
            return -3;
        } else if (obj->Signal_Cnt > 12000 && obj->Signal_Cnt <= 14000) {
            return 4;
        } else if (obj->Signal_Cnt > 14000 && obj->Signal_Cnt <= 16000) {
            return -4;
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}
