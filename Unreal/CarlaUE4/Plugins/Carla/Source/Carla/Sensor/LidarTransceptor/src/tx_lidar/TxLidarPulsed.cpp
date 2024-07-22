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
    return 0;
};

/*----------------------------------------------------------------------------*/
vector<float> TxLidarPulsed::run()
{
  out_bits.clear();
  //cout << "DEBUG1" << endl;
  //cout << MAX_RANGE << LIGHT_SPEED << FS << NOS << endl;
  int LEN_TOTAL = int((2*MAX_RANGE/LIGHT_SPEED)*FS*NOS); // Tiempo Máximo * Frecuencia de Muestreo * Sobremuestreo

  for (int i = 0; i<LEN_TOTAL; i++)
    if ( i < int(TAU_SIGNAL*FS*NOS) ) // Tiempo del pulso
      out_bits.push_back(POWER_TX);
    else
      out_bits.push_back(0);

  return out_bits;
}

/*----------------------------------------------------------------------------*/
void TxLidarPulsed::exposeVar(){

}
