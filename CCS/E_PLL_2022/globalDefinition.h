/*
 * globalDefinition.h
 *
 *  Created on: 14 de mar de 2022
 *      Author: cleiton Magalhães Freitas
 */

#ifndef GLOBALDEFINITION_H_
#define GLOBALDEFINITION_H_

// Some important definitions
#define Tsamp 5e-5

/* Global Variables */
static float v_in = 0.2;
static float w_in[2] = {376.991118431,376.991118431};
static float wt_in[2] = {0,0};


/* variáveis da fonte de tensão */
static float V1p = 5;
static float V3p = 0;
static float V5p = 0;
static float V7p = 0;

static float f1 = 60;
static float phi1 = 0;


/* Parâmetros do PLL modificado */
// control gains
#define ki_1 2000
#define kp_1 10
#define Ka_1 50
#define K_1 2

// Condições iniciais
#define wo_1 320

// Variáveis de teste
long int flag=0;

/* Variávies do PLL modificado */
static float sda_valfa;
static float sda_valfa_old;


static float sda_vbeta;
static float sda_vbeta_old;

static float w;
static float w_old;

static float vbeta;

static float vq;
static float vq_old;

static float pll_alfa;
static float pll_alfa_old;

static float pll_beta;
static float pll_beta_old;

static float teta;
static float teta_old;

static float acum_w;
static float acum_w_old;

static float wp;

static float wi;

static float erro_amp;
static float erro_amp_old;

static float amp;
static float amp_old;

static float erro1_sogi;
static float erro1_sogi_old;

static float erro2_sogi;
static float erro2_sogi_old;


#endif /* GLOBALDEFINITION_H_ */
