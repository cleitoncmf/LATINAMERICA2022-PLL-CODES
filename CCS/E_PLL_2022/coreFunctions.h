/*
 * coreFunctions.h
 *
 *  Created on: 14 de mar de 2022
 *      Author: cleiton Magalhães Freitas
 */

#ifndef COREFUNCTIONS_H_
#define COREFUNCTIONS_H_




void SinGenerator(float *wt, float *vsin, float *wfreq, float freq, float phi, float Am, float Ts);
void DESLOCA(float *IN, int N);
void INTEGRA(float *E, float *S, float min, float max, float Ts);


void SinGenerator_h(float *wt,
                    float *vsin,
                    float *wfreq,
                    float freq,
                    float phi,
                    float Am,
                    float phi3,
                    float Am3,
                    float phi5,
                    float Am5,
                    float phi7,
                    float Am7,
                    float Ts);

void EPLL_mod(float *sda_valfa,
              float *sda_valfa_old,
              float *sda_vbeta,
              float *sda_vbeta_old,
              float *w,
              float *w_old,
              float *vbeta,
              float *vq,
              float *vq_old,
              float *pll_alfa,
              float *pll_alfa_old,
              float *pll_beta,
              float *pll_beta_old,
              float *teta,
              float *teta_old,
              float *acum_w,
              float *acum_w_old,
              float *wp,
              float *wi,
              float *erro_amp,
              float *erro_amp_old,
              float *amp,
              float *amp_old,
              float *vin,
              float Ts,
              float ki,
              float kp,
              float wo,
              float Ka);

void SOGI_EPLL_mod(float *sda_valfa,
                   float *sda_valfa_old,
                   float *sda_vbeta,
                   float *sda_vbeta_old,
                   float *w,
                   float *w_old,
                   float *vbeta,
                   float *vq,
                   float *vq_old,
                   float *pll_alfa,
                   float *pll_alfa_old,
                   float *pll_beta,
                   float *pll_beta_old,
                   float *teta,
                   float *teta_old,
                   float *acum_w,
                   float *acum_w_old,
                   float *wp,
                   float *wi,
                   float *erro_amp,
                   float *erro_amp_old,
                   float *amp,
                   float *amp_old,
                   float *vin,
                   float *erro1_sogi,
                   float *erro1_sogi_old,
                   float *erro2_sogi,
                   float *erro2_sogi_old,
                   float Ts,
                   float ki,
                   float kp,
                   float wo,
                   float Ka,
                   float K);


#endif /* COREFUNCTIONS_H_ */
