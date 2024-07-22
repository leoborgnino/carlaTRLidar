/*-----------------------------------------------------------------------------
  -- Proyecto      : Lidar PCM
  -------------------------------------------------------------------------------
  -- Archivo       : RxLidarpulsed.h
  -- Organizacion  : Fundacion Fulgor
  -- Fecha         : 13 de Junio 2023
  -------------------------------------------------------------------------------
  -- Descripcion   : Receptor de Lidar Pulsado
  -------------------------------------------------------------------------------
  -- Autor         : Leandro Borgnino
  -------------------------------------------------------------------------------
  -- Copyright (C) 2023 Fundacion Fulgor  All rights reserved
  -------------------------------------------------------------------------------
  -- $Id$
  -------------------------------------------------------------------------------*/

#ifndef RxLidarPulsed_H
#define RxLidarPulsed_H

// Includes common C++
#include <fstream>
#include <vector>
#include <iostream>
#include <stdlib.h> 
#include <ctime>
#include <math.h>
#include <map>
#include <complex>
#include <random>
#include <chrono>
#include <algorithm>


// Includes Propios
#include "Carla/Sensor/LidarTransceptor/src/common/constants.h"
using namespace std;

/*----------------------------------------------------------------------------*/
class RxLidarPulsed
{
  /*-----------------------------------------------------------------------*/
public:
  RxLidarPulsed();
  ~RxLidarPulsed();

  /**El metodo init, utiliza la clase loadSettings para determinar los valores de las
     variables que utiliza. */
  int init(parametersLiDAR *params);
  vector<float> run(vector<float> input_rx_from_tx, vector<float> input_rx_from_channel);    

  // Interfaces
  vector <float> out_bits;

  /*-----------------------------------------------------------------------*/
private:
  void exposeVar();
  std::vector<float> convolucion(const std::vector<float>& signal, const std::vector<float>& kernel);
  // Params
  int MAX_RANGE,NOS;
  double FS, POWER_RX, RPD, NOISE;
  bool DEBUG_RX;
  
  // Variables
  double noise_power;

};
#endif
