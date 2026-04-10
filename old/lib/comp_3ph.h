    //     // if (compensation_current == 1) {
    //     //     int16_t dt_correction_sincos =
    //     //         deadtime_compensate_sqrt(current_ab_filt.cos + current_ab_filt.sin, voltage_ab.cos + voltage_ab.sin,
    //     //                                  deadzone_sincos, i_deadband_sincos, dead_time_duty);
    //     //     int16_t dt_correction_sin =
    //     //         deadtime_compensate_sqrt(current_ab_filt.sin, voltage_ab.sin, deadzone, i_deadband, dead_time_duty) +
    //     //         dt_correction_sincos;
    //     //     int16_t dt_correction_cos =
    //     //         deadtime_compensate_sqrt(current_ab_filt.cos, voltage_ab.cos, deadzone, i_deadband, dead_time_duty) +
    //     //         dt_correction_sincos;

    //     //     voltage_ab.sin += dt_correction_sin;
    //     //     voltage_ab.cos += dt_correction_cos;
    //     // }


    //         /**
    //  * @param current       Текущий ток (int16_t)
    //  * @param threshold     Пороговое значение тока (int16_t, например 100)
    //  * @param compensation  Полная величина компенсации (int16_t)
    //  * @return              Скорректированная компенсация
    //  */
    // int16_t deadtime_compensate_sqrt(int16_t current, int16_t voltage, int16_t threshold, int16_t i_dband, int16_t compensation) {
    //     if (threshold <= 0 || compensation == 0) {
    //         return 0;
    //     }
    //     int32_t cur = current;

    //     int8_t current_sign = (cur >= 0) ? 1 : -1;
    //     int8_t voltage_sign = (voltage >= 0) ? 1 : -1;

    //     int32_t abs_current = cur * current_sign;

    //     if (abs_current >= threshold) {
    //         return current_sign * compensation;
    //     } else if (abs_current <= i_dband) {
    //         return 0;
    //     } else {
    //         // Переводим в float для расчета корня
    //         float ratio = (float)(abs_current - i_dband) / (float)(threshold - i_dband);
    //         // sqrtf() компилируется напрямую в инструкцию VSQRT.F32
    //         // при правильных настройках FPU (-mfloat-abi=hard -mfpu=fpv4-sp-d16)
    //         float f = sqrtf(ratio);
    //         // Масштабируем и возвращаем в целые числа
    //         int16_t scaled_comp = (int16_t)((float)compensation * f);
    //         return current_sign * scaled_comp;
    //     }
    // }