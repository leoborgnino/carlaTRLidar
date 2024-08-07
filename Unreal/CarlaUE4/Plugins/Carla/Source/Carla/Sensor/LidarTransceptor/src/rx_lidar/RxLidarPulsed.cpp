/*-----------------------------------------------------------------------------
  -- Proyecto      : Lidar PCM
  -------------------------------------------------------------------------------
  -- Archivo       : RxLidarPulsed.cpp
  -- Organizacion  : Fundacion Fulgor
  -- Fecha         : 13 de junio 2023
  -------------------------------------------------------------------------------
  -- Descripcion   : Receptor de Lidar Pulsado
  -------------------------------------------------------------------------------
  -- Autor         : Leandro Borgnino
  -------------------------------------------------------------------------------
  -- Copyright (C) 2023 Fundacion Fulgor  All rights reserved
  -------------------------------------------------------------------------------
  -- $Id$
  -------------------------------------------------------------------------------*/

#include "RxLidarPulsed.h"

/*----------------------------------------------------------------------------*/
RxLidarPulsed::RxLidarPulsed()
{
    
  printf("RxLidarPulsed :: Has been created\n");

};

/*----------------------------------------------------------------------------*/
RxLidarPulsed::~RxLidarPulsed(){};

/*----------------------------------------------------------------------------*/
int  RxLidarPulsed::init(parametersLiDAR *params){
  /*!Se utiliza para realizar la carga de parametros y la configuracion 
    inicial de las variables*/    
  MAX_RANGE = params->MAX_RANGE;

  DEBUG_RX = params->DEBUG_GLOBAL;
  FS = params->RX_FS;
  NOS = params->RX_NOS;
  POWER_RX = params->PRX;    
  RPD = params->RPD;

  noise_power = RPD;//Q_ELECT/RPD*FS; // FIXME PLS UWU
  return 0;
};

/*----------------------------------------------------------------------------*/
vector<float> RxLidarPulsed::run(vector<float> input_rx_from_tx, vector<float> input_rx_from_channel)
{
  vector<float> noise_vector;

  // Generador random
  unsigned seed = chrono::system_clock::now().time_since_epoch().count();
  const float mean = 0.0;
  const float stddev = noise_power;
  default_random_engine generator(seed);
  normal_distribution<float> dist(mean, stddev);

  if (DEBUG_RX)
    {
      cout << "************************" << endl;
      cout << "** Datos del Receptor **" << endl;
      cout << "************************" << endl;
      cout << "Noise Power Gain: " << noise_power << endl;
    }
  
  // AFE
  out_bits = input_rx_from_channel;
  for (unsigned int ii = 0; ii < input_rx_from_channel.size(); ii++)
    out_bits[ii] = input_rx_from_channel[ii] + sqrt(noise_power)*dist(generator);

  // MF
  vector<float> matched_filter = input_rx_from_tx;
  reverse(matched_filter.begin(), matched_filter.end());
  out_bits = convolucion(matched_filter,out_bits);
  
  return out_bits;
}

/*----------------------------------------------------------------------------*/
void RxLidarPulsed::exposeVar(){

}

std::vector<float> RxLidarPulsed::convolucion(const std::vector<float>& signal, const std::vector<float>& kernel) {
    std::vector<float> result(signal.size() + kernel.size() - 1, 0.0);

    for (size_t i = 0; i < signal.size(); ++i) {
        for (size_t j = 0; j < kernel.size(); ++j) {
            result[i + j] += signal[i] * kernel[j];
        }
    }

    return result;
}

//std::vector<float> RxLidarPulsed::convolucion(const std::vector<float>& signal, const std::vector<float>& kernel) {
//    std::vector<float> result(signal.size(), 0.0);
//    int kernelRadius = kernel.size() / 2;
//
//    for (int i = 0; i < int(signal.size()); ++i) {
//        for (int j = -kernelRadius; j <= kernelRadius; ++j) {
//            int index = i + j;
//            if (index >= 0 && index < signal.size()) {
//                result[i] += signal[index] * kernel[j + kernelRadius];
//            }
//        }
//    }
//
//    return result;
//}
