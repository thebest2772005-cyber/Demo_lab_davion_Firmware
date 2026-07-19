#include "encoder.h"

/**
  * @brief  Kh?i t?o c?u trúc Encoder và d?ng b? Counter ban d?u
  * @param  enc: Con tr? t?i struct Encoder
  * @param  TIMx: Con tr? t?i Timer ph?n c?ng (Ví d?: TIM2)
  */
void Encoder_Init(Encoder_t *enc, TIM_TypeDef *TIMx)
{
    enc->Instance = TIMx;
    enc->encoder_position = 0;
    enc->prev_cnt = (int16_t)TIMx->CNT; //
    
    enc->speed_rpm = 0.0f;
    enc->speed_rad_s = 0.0f;
    enc->velocity_m_s = 0.0f;
    enc->angle_deg = 0.0f;
}

/**
  * @brief  Hàm x? lý và c?p nh?t toàn b? tín hi?u Encoder (G?i d?nh k? theo SAMPLE_TIME)
  * @param  enc: Con tr? t?i struct Encoder
  */
void Encoder_Update(Encoder_t *enc)
{
    uint16_t now = (uint16_t)enc->Instance->CNT;

    int16_t delta = (int16_t)(now - (uint16_t)enc->prev_cnt);

    enc->encoder_position += delta;
    enc->prev_cnt = now;

    // RPM
    enc->speed_rpm = ((float)delta * 60.0f) / (COUNTS_PER_REV * SAMPLE_TIME_S);
    
    // Rad/s
    enc->speed_rad_s = enc->speed_rpm * 0.104719755f; 
    
    // m/s
    enc->velocity_m_s = enc->speed_rad_s * WHEEL_RADIUS_M;

    // Angle
    int32_t pos_modulo = enc->encoder_position % (int32_t)COUNTS_PER_REV;
    if (pos_modulo < 0) {
        pos_modulo += (int32_t)COUNTS_PER_REV;
    }
    enc->angle_deg = (float)pos_modulo * (360.0f / COUNTS_PER_REV);
}