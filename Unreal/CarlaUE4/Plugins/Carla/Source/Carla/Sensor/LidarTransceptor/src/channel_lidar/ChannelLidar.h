/*-----------------------------------------------------------------------------
  -- Proyecto      : LiDAR Pulsado C++
  -------------------------------------------------------------------------------
  -- Archivo       : ChannelLidar.h
  -- Organizacion  : Fundacion Fulgor
  -- Fecha         : 13 de Junio 2023
  -------------------------------------------------------------------------------
  -- Descripcion   : Canal de LiDAR Pulsado
  -------------------------------------------------------------------------------
  -- Autor         : Leandro Borgnino
  -------------------------------------------------------------------------------
  -- Copyright (C) 2023 Fundacion Fulgor  All rights reserved
  -------------------------------------------------------------------------------
  -- $Id$
  -------------------------------------------------------------------------------*/

#ifndef ChannelLidar_H
#define ChannelLidar_H

// Includes common C++
#include <fstream>
#include <vector>
#include <iostream>
#include <stdlib.h> 
#include <ctime>
#include <cmath>
#include <map>
#include <complex>
#include <algorithm>

// Includes Propios
#include "Carla/Sensor/LidarTransceptor/src/common/constants.h"

using namespace std;

/*----------------------------------------------------------------------------*/
class ChannelLidar
{
  /*-----------------------------------------------------------------------*/
 public:
  ChannelLidar();
  ~ChannelLidar();
  
  /**El metodo init, utiliza la clase parametersLiDAR para determinar los valores de las
     variables que utiliza. */
  int init(parametersLiDAR *params);
  vector<float> run(vector<float> channel_input, float range, float rho, float angle_inc); 
  
  
  /*-----------------------------------------------------------------------*/
 private:
  void exposeVar();
  
  // Params
  double ARX, FS, LAMBDA0;
  int NOS;
  bool DEBUG_CH;
  
  // Variables

};
#endif
