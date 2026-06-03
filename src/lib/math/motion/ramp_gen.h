// #pragma once

// #include "../../common.h" 

// #include "motion_executor.h"

// // Убедитесь, что класс MotionExecutor доступен (подключен) перед этим файлом
// // #include "MotionExecutor.h" 

// /// @brief Входные параметры для генерации S-кривой
// struct SCurveParams {
//     int32_t pos_start; // Начальное положение в Q16.16
//     int32_t pos_end;   // Конечное положение в Q16.16
//     float vel_start;   // Начальная скорость (RPS)
//     float vel_end;     // Конечная скорость (RPS)
//     float acc_max;     // Максимальное ускорение (RPS/s)
//     float dec_max;     // Максимальное замедление (RPS/s), должно быть > 0
//     float j1;          // Рывок для нарастания ускорения
//     float j2;          // Рывок для спада ускорения
//     float j3;          // Рывок для нарастания замедления
//     float j4;          // Рывок для спада замедления
//     float vel_lim;     // Ограничение скорости на профиле (целевая скорость этапа)
//     float threshold;   // Минимальное время фазы в секундах (например, 1e-6f)
// };

// class SCurvePlanner {
// private:
//     struct Phase {
//         float t_sec = 0.0f; 
//         float ds = 0.0f;    
//         float v0 = 0.0f;    
//         float a0 = 0.0f;    
//         float j0 = 0.0f;    
//     };

//     struct HalfCurve {
//         Phase p1, p2, p3;
//         float total_s = 0.0f;
//     };

//     static HalfCurve calc_half_s(float v_init, float v_exit, float j_init, float j_exit, float a_max, float threshold) {
//         HalfCurve res; 

//         // ФАЗА 1 - Нарастание ускорения
//         float t01 = 0.0f, s01 = 0.0f, v1 = v_init;
//         float p1_dur = a_max / j_init;
//         if (p1_dur > threshold) {
//             t01 = p1_dur;
//             float dv = j_init * t01 * t01 * 0.5f;
//             v1 = v_init + dv;
//             s01 = t01 * (v_init + dv / 3.0f);
//         }

//         // ФАЗА 3 - Спад ускорения
//         float v2 = v_exit;
//         float t23 = 0.0f, s23 = 0.0f;
//         float p3_dur = a_max / j_exit;
//         if (p3_dur > threshold) {
//             t23 = p3_dur;
//             float dv = -j_exit * t23 * t23 * 0.5f;
//             v2 = v_exit + dv; 
//             s23 = t23 * (v_exit + dv / 3.0f);
//         }

//         // ФАЗА 2 - Постоянное ускорение
//         float t12 = 0.0f, s12 = 0.0f;
//         float p2_dur = (v2 - v1) / a_max;
//         if (p2_dur > threshold) {
//             t12 = p2_dur;
//             s12 = t12 * (v1 + a_max * t12 * 0.5f);
//         }

//         res.total_s = s01 + s12 + s23;
        
//         res.p1 = { t01, s01, v_init, 0.0f, j_init };
//         res.p2 = { t12, s12, v1,    a_max, 0.0f };
//         res.p3 = { t23, s23, v2,    a_max, -j_exit };

//         return res;
//     }

// public:
//     /// @brief Рассчитывает 7-фазный профиль и сразу пушит готовые сегменты в Executor
//     /// @tparam N Размер буфера экзекутора
//     /// @return true, если все сегменты успешно добавлены. false, если буфер переполнился.
//     template <size_t N>
//     static bool generate_and_push(const SCurveParams& p, MotionExecutor<N>& executor) {
//         // Debug: log input parameters
//         TP_LOG_VALS("SCurve params:", p.pos_start, p.pos_end, p.vel_start, p.vel_end, p.vel_lim);
//         TP_LOG_VALS("SCurve limits:", p.acc_max, p.dec_max, p.j1, p.j2, p.j3, p.j4, p.threshold);
        
//         Phase phases[7]; // Локальный C-массив только для промежуточных float расчетов

//         float path_total = static_cast<float>(p.pos_end - p.pos_start) / 65536.0f;
        
//         // Разгон (Фазы 0, 1, 2)
//         float s03 = 0.0f;
//         if (p.vel_start != p.vel_lim) {
//             HalfCurve h1 = calc_half_s(p.vel_start, p.vel_lim, p.j1, p.j2, p.acc_max, p.threshold);
//             phases[0] = h1.p1;
//             phases[1] = h1.p2;
//             phases[2] = h1.p3;
//             s03 = h1.total_s;
//         }

//         // Торможение (Фазы 4, 5, 6)
//         float s47 = 0.0f;
//         if (p.vel_end != p.vel_lim) {
//             HalfCurve h2 = calc_half_s(p.vel_lim, p.vel_end, -p.j3, -p.j4, -p.dec_max, p.threshold);
//             phases[4] = h2.p1;
//             phases[5] = h2.p2;
//             phases[6] = h2.p3;
//             s47 = h2.total_s;
//         }

//         // Постоянная скорость (Фаза 3)
//         float s34 = path_total - s03 - s47;
        
//         // ИСПРАВЛЕНИЕ: Проверяем расчетное время, а не знак пути.
//         // Для реверса s34 < 0 и vel_lim < 0, поэтому t34 будет > 0.
//         if (p.vel_lim != 0.0f) {
//             float t34 = s34 / p.vel_lim;
            
//             // Если времени хватает (профиль не вырождается в треугольник)
//             if (t34 > p.threshold) {
//                 phases[3] = { t34, s34, p.vel_lim, 0.0f, 0.0f };
//             }
//         }

//         int32_t current_pos = p.pos_start;
//         bool all_pushed = true;

//         int segments_added = 0; // Counter for actually added segments

//         // Формируем команды и сразу отправляем в буфер
//         for (int i = 0; i < 7; ++i) {
//             // Debug: log phase data before processing
//             TP_LOG_VALS("Phase data:", i, phases[i].t_sec, phases[i].ds, phases[i].v0, phases[i].a0, phases[i].j0);
            
//             // Игнорируем фазы с нулевой (или микроскопической) длительностью
//             if (phases[i].t_sec <= p.threshold) {
//                 continue; 
//             }

//             MotionSegment seg;
//             seg.mode = 0; // TODO: Установить маску режима (например, DO_POS | DO_VEL)
//             seg.pos = current_pos;
//             seg.vel = phases[i].v0;
//             seg.acc = phases[i].a0;
//             seg.jrk = phases[i].j0;
//             seg.trq = 0.0f; // Ограничение момента по умолчанию

//             // Перевод времени в микросекунды с округлением
//             seg.t = static_cast<uint32_t>(phases[i].t_sec * 1000000.0f + 0.5f);

//             // Накапливаем дельту для стартовой позиции следующего сегмента
//             current_pos += static_cast<int32_t>(phases[i].ds * 65536.0f);

//             // Пушим в кольцевой буфер экзекутора
//             if (!executor.add_segment(seg)) {
//                 all_pushed = false;
//                 break; // Буфер переполнен, прерываем добавление
//             }
//             segments_added++; // Increment counter for successfully added segment
            
//             // Debug: log each segment being added to the buffer
//             TP_LOG_VALS("Added segment to buffer:", seg.t, seg.mode, seg.pos, seg.vel, seg.acc, seg.jrk, seg.trq);
//         }

//         return all_pushed && (segments_added > 0); // Return false if no segments were added
//     }
// };
