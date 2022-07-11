/*
 * coreFunctions.c
 *
 *  Created on: 14 de mar de 2022
 *      Author: cleiton Magalhães Freitas
 */


#include<math.h>
#include"coreFunctions.h"



//! *wt - pointer to store the voltage angle (the variable must be a size-2 array)
//! *vsin - pointer to store the voltage (must be a scalar)
//! f - frequency of the signal
//! phi - displacement angle of the signal
//! Am - Amplitude of the signal
void SinGenerator(float *wt, float *vsin, float *wfreq, float freq, float phi, float Am, float Ts){
    //float wfreq[2] = {2*3.14159265359*freq,2*3.14159265359*freq};

    DESLOCA(wfreq, 1);
    *wfreq = 6.28318530718*freq;


    DESLOCA(wt, 1);

    INTEGRA(wfreq, wt, 0, 6.28318530718, Ts);

    *vsin = Am*sinf(*wt);

}


//! *wt - pointer to store the voltage angle (the variable must be a size-2 array)
//! *vsin - pointer to store the voltage (must be a scalar)
//! f - frequency of the signal
//! phi - displacement angle of the signal
//! Am - Amplitude of the signal
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
                    float Ts){
    //float wfreq[2] = {2*3.14159265359*freq,2*3.14159265359*freq};

    DESLOCA(wfreq, 1);
    *wfreq = 6.28318530718*freq;


    DESLOCA(wt, 1);

    INTEGRA(wfreq, wt, 0, 6.28318530718, Ts);

    *vsin = Am*sinf(*wt+phi) + Am3*sinf(3*(*wt)+phi3)+ Am5*sinf(5*(*wt)+phi5)+Am7*sinf(7*(*wt)+phi7);

}



/**
*
* Função que desloca os elementos de um array N posições a direita
*
* OBS: esta função só opera sobre os N primeios elementos de um vetor.
* Caso o vetor possua mais de N elementos, os elementos restantes não serão deslocados.
*
* A variável de entrada IN é passados por referência.
* A função não tem retorno, a variavel de entrada é modificada internamente.
*
* @param IN - Entrada e saída do função
*
*/
void DESLOCA(float *IN, int N){
    int i;
    for(i=1;i<=N;i++){
        *(IN + N-i+1) = *(IN + N-i);
    }
}






/* Função que implementa o integrador discreto com constante de tempo unitária
 *
 * @param E - ponteiro para a Entrada do integrador
 * @param S - ponteiro para a Saida
 * @param Fs - Frequência de amostragem
 * @return
 */
void INTEGRA(float *E, float *S, float min, float max, float Ts){


    if(*(S+1)<=max & *(S+1)>=min){
        *S = (0.5*Ts)*(*E + *(E+1)) + *(S+1);
    }
    else{
        *S = 0;
    }


}



/*
 * Função para implementar o EPLL modificado
 *
 *
 *
 * */
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
              float Ka){




    // LOOP do APF
    *sda_valfa_old = *sda_valfa;
    *sda_valfa = *vin;

    *sda_vbeta_old = *sda_vbeta;
    *sda_vbeta = ( (2 - Ts*(*w_old))/(2 + Ts*(*w_old)) )*(*sda_valfa + *sda_vbeta_old) - *sda_valfa_old;

    *vbeta = -*sda_vbeta;



     // Malha de Fase do E-PLL modificado
     *vq_old = *vq;
     *vq = (*sda_valfa - *pll_alfa_old)*cosf(*teta_old) + (*vbeta - *pll_beta_old)*sinf(*teta_old);

     *acum_w_old = *acum_w;
     *acum_w = *acum_w_old + (*vq + *vq_old)*ki*Ts*0.5;

     *wp = kp*(*vq);

     *wi = wo + (*acum_w);

     *w_old = *w;
     *w = *wp + *wi;

     *teta_old = *teta;
     *teta = *teta_old + (*w + *w_old)*Ts*0.5;

     if (*teta > 6.283185307179586){
         *teta = 0;
     }


     // Malha de Amplitude do E-PLL modificado
     *erro_amp_old = *erro_amp;
     *erro_amp = (*sda_valfa - *pll_alfa_old)*sinf(*teta_old) - (*vbeta - *pll_beta_old)*cosf(*teta_old);

     *amp_old = *amp;
     *amp = *amp_old + (*erro_amp + *erro_amp_old)*Ka*Ts*0.5;


     *pll_alfa_old = *pll_alfa;
     *pll_alfa = *amp*sinf(*teta);

     *pll_beta_old = *pll_beta;
     *pll_beta = -*amp*cosf(*teta);









}





/*
 * Função para implementar o EPLL modificado com o SOGI
 *
 *
 *
 * */
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
                   float K){




    // LOOP do SOGI
    *sda_valfa_old = *sda_valfa;
    *sda_vbeta_old = *sda_vbeta;

    *erro1_sogi_old = *erro1_sogi;
    *erro2_sogi_old = *erro2_sogi;

    *erro1_sogi = (  K*( *vin - *sda_valfa_old )  - *sda_vbeta_old )* (*w_old);
    *sda_valfa = *sda_valfa_old + (0.5*Ts)*(*erro1_sogi + *erro1_sogi_old);

    *erro2_sogi = (*w_old)*(*sda_valfa_old);
    *sda_vbeta = *sda_vbeta_old + (0.5*Ts)*(*erro2_sogi + *erro2_sogi_old);

    *vbeta = *sda_vbeta;


     // Malha de Fase do E-PLL modificado
     *vq_old = *vq;
     *vq = (*sda_valfa - *pll_alfa_old)*cosf(*teta_old) + (*vbeta - *pll_beta_old)*sinf(*teta_old);

     *acum_w_old = *acum_w;
     *acum_w = *acum_w_old + (*vq + *vq_old)*ki*Ts*0.5;

     *wp = kp*(*vq);

     *wi = wo + (*acum_w);

     *w_old = *w;
     *w = *wp + *wi;

     *teta_old = *teta;
     *teta = *teta_old + (*w + *w_old)*Ts*0.5;

     if (*teta > 6.283185307179586){
         *teta = 0;
     }


     // Malha de Amplitude do E-PLL modificado
     *erro_amp_old = *erro_amp;
     *erro_amp = (*sda_valfa - *pll_alfa_old)*sinf(*teta_old) - (*vbeta - *pll_beta_old)*cosf(*teta_old);

     *amp_old = *amp;
     *amp = *amp_old + (*erro_amp + *erro_amp_old)*Ka*Ts*0.5;


     *pll_alfa_old = *pll_alfa;
     *pll_alfa = *amp*sinf(*teta);

     *pll_beta_old = *pll_beta;
     *pll_beta = -*amp*cosf(*teta);



}

