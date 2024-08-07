/*-----------------------------------------------------------------------------
  -- Proyecto      : Lidar PCM
  -------------------------------------------------------------------------------
  -- Archivo       : TxLidarPulsed.h
  -- Organizacion  : Fundacion Fulgor
  -- Fecha         : 13 de Junio 2023
  -------------------------------------------------------------------------------
  -- Descripcion   : Transmisor de LiDAR Pulsado
  -------------------------------------------------------------------------------
  -- Autor         : Leandro Borgnino
  -------------------------------------------------------------------------------
  -- Copyright (C) 2023 Fundacion Fulgor  All rights reserved
  -------------------------------------------------------------------------------
  -- $Id$
  -------------------------------------------------------------------------------*/

#ifndef TxLidarPulsed_H
#define TxLidarPulsed_H

// Includes common C++
#include <fstream>
#include <vector>
#include <iostream>
#include <stdlib.h> 
#include <ctime>
#include <math.h>
#include <map>
#include <complex>

// Includes Propios
#include "Carla/Sensor/LidarTransceptor/src/common/constants.h"

using namespace std;

/*----------------------------------------------------------------------------*/
class TxLidarPulsed
{
  /*-----------------------------------------------------------------------*/
public:
  TxLidarPulsed();
  ~TxLidarPulsed();

  /**El metodo init, utiliza la clase loadSettings para determinar los valores de las
     variables que utiliza. */
  int init(parametersLiDAR *params);
  vector<float> run();    

  // Interfaces
  vector <float> out_bits;

  /*-----------------------------------------------------------------------*/
private:
  void exposeVar();

  std::vector<float> rectangular_pulse(float I_max, float T_pulso, float LEN_TOTAL, float FS, int NOS );
  std::vector<float> gaussian_pulse(float I_max, float T_pulso, int LEN_TOTAL, float FS, int NOS);

  // Params
  int MAX_RANGE,NOS, PULSE_SHAPE;
  double TAU_SIGNAL, FS, POWER_TX;
  
  // Variables

};
#endif
