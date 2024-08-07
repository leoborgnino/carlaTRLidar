/*-----------------------------------------------------------------------------
-- Proyecto      : Lidar PCM
-------------------------------------------------------------------------------
-- Archivo       : TxLidarPulsed.cpp
-- Organizacion  : Fundacion Fulgor
-- Fecha         : 13 de junio 2023
-------------------------------------------------------------------------------
-- Descripcion   : Transmisor de LiDAR Pulsado
-------------------------------------------------------------------------------
-- Autor         : Leandro Borgnino
-------------------------------------------------------------------------------
-- Copyright (C) 2023 Fundacion Fulgor  All rights reserved
-------------------------------------------------------------------------------
-- $Id$
-------------------------------------------------------------------------------*/

#include "TxLidarPulsed.h"

/*----------------------------------------------------------------------------*/
TxLidarPulsed::TxLidarPulsed()
{
    
    printf("TxLidarPulsed :: Has been created\n");

};

/*----------------------------------------------------------------------------*/
TxLidarPulsed::~TxLidarPulsed(){};

/*----------------------------------------------------------------------------*/
int  TxLidarPulsed::init(parametersLiDAR *params){
    /*!Se utiliza para realizar la carga de parametros y la configuracion 
      inicial de las variables*/    
    MAX_RANGE = params->MAX_RANGE;
    TAU_SIGNAL = params->TAU_SIGNAL;
    FS = params->TX_FS;
    NOS = params->TX_NOS;
    POWER_TX = params->PTX;
    PULSE_SHAPE = params->PULSE_SHAPE;
    return 0;
};

/*----------------------------------------------------------------------------*/
vector<float> TxLidarPulsed::run()
{
  out_bits.clear();
  //cout << "DEBUG1" << endl;
  //cout << MAX_RANGE << LIGHT_SPEED << FS << NOS << endl;
  int LEN_TOTAL = int((2*MAX_RANGE/LIGHT_SPEED)*FS*NOS); // Tiempo Máximo * Frecuencia de Muestreo * Sobremuestreo

  if (PULSE_SHAPE == 0)
    out_bits = gaussian_pulse(POWER_TX, TAU_SIGNAL, LEN_TOTAL, FS, NOS);
  else
    out_bits = rectangular_pulse(POWER_TX, TAU_SIGNAL, LEN_TOTAL, FS, NOS);
  
  return out_bits;
}

/*----------------------------------------------------------------------------*/
void TxLidarPulsed::exposeVar(){

}

std::vector<float> TxLidarPulsed::gaussian_pulse(float I_max, float T_pulso, int LEN_TOTAL, float FS, int NOS) {
    // Calcular sigma a partir de T_pulso
    float sigma = T_pulso / 2.355;

    // Vector para almacenar los valores de la signal
    std::vector<float> signal;

    // Calcular la signal para cada punto de tiempo t
    for (int t = 0; t < LEN_TOTAL; t++) {
      float s_t = I_max * std::exp(-std::pow(t/(FS*NOS)-(T_pulso+T_pulso/2), 2) / (2 * std::pow(sigma, 2)));
        signal.push_back(s_t);
    }

    return signal;
}


std::vector<float> TxLidarPulsed::rectangular_pulse(float I_max, float T_pulso, float LEN_TOTAL, float FS, int NOS ) {

    // Vector para almacenar los valores de la signal
    std::vector<float> signal;

    // Calcular la signal para cada punto de tiempo t
    for (int i = 0; i<LEN_TOTAL; i++)
      if ( (i >= int(FS*NOS*(T_pulso/2)) ) && (i <= int((3*T_pulso/2)*FS*NOS)) ) // Tiempo del pulso
	signal.push_back(I_max);
      else
	signal.push_back(0);

    return signal;
}
