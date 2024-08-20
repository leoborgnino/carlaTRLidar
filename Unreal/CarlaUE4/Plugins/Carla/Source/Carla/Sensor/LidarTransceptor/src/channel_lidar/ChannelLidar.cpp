/*-----------------------------------------------------------------------------
-- Proyecto      : Lidar PCM
-------------------------------------------------------------------------------
-- Archivo       : ChannelLidar.cpp
-- Organizacion  : Fundacion Fulgor
-- Fecha         : 13 de junio 2023
-------------------------------------------------------------------------------
-- Descripcion   : Canal de LiDAR Pulsado
-------------------------------------------------------------------------------
-- Autor         : Leandro Borgnino
-------------------------------------------------------------------------------
-- Copyright (C) 2023 Fundacion Fulgor  All rights reserved
-------------------------------------------------------------------------------
-- $Id$
-------------------------------------------------------------------------------*/

#include "ChannelLidar.h"

/*----------------------------------------------------------------------------*/
ChannelLidar::ChannelLidar()
{
    
    printf("ChannelLidar :: Has been created\n");

};

/*----------------------------------------------------------------------------*/
ChannelLidar::~ChannelLidar(){};

/*----------------------------------------------------------------------------*/
int  ChannelLidar::init(parametersLiDAR *params){
    /*!Se utiliza para realizar la carga de parametros y la configuracion 
      inicial de las variables*/
    LAMBDA0 = params->LAMBDA0;
    DEBUG_CH = params->DEBUG_GLOBAL;
    FS = params->CH_FS;
    NOS = params->CH_NOS;
    ARX = params->ARX;
    
    return 0;
};

/*----------------------------------------------------------------------------*/
vector<float> ChannelLidar::run(vector<float> channel_input, float range, float rho)
{
  float delay = 2*range/LIGHT_SPEED;
  int delay_samples = delay*FS*NOS;

  float meas_delay = delay_samples/FS;
  float meas_range = meas_delay*LIGHT_SPEED/2;

  float power_gain  = rho;
  //double delta_phase = LIGHT_SPEED/LAMBDA0*meas_delay;

  if (DEBUG_CH)
    {
      cout << "************************" << endl;
      cout << "*** Datos del Canal   **" << endl;
      cout << "************************" << endl;
      cout << "Channel Power Gain: " << power_gain << endl;
    }

  vector<float> channel_output = channel_input;
  
  // Multiplicación del vector por la ganancia del canal
  transform(channel_output.begin(), channel_output.end(), channel_output.begin(),
	          [&power_gain](float element) { return element *= power_gain; });

  // Desplazamiento según el retardo
  // Warning: Posible bug porque es un shift circular
  //rotate(channel_output.begin(), channel_output.begin()+channel_output.size()-delay_samples, channel_output.end());
  channel_output.insert(channel_output.begin(), delay_samples, 0);
  channel_output.erase(channel_output.end() - delay_samples, channel_output.end());

  // Warning: Faltaría el desplazamiento de la fase por ahora solo real
  
  return channel_output;
}

/*----------------------------------------------------------------------------*/
void ChannelLidar::exposeVar(){

}
