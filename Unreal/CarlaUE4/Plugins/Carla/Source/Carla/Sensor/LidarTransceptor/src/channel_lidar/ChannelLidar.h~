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
#include "constants.h"
#include "ffModule/loadSettings.h"

using namespace std;

/*----------------------------------------------------------------------------*/
class ChannelLidar
{
  /*-----------------------------------------------------------------------*/
 public:
  ChannelLidar();
  ~ChannelLidar();
  
  /**El metodo init, utiliza la clase loadSettings para determinar los valores de las
     variables que utiliza. */
  int init(loadSettings *params);
  vector<double> run(vector<double> channel_input, double range, double rho, double angle_inc); 
  
  // Interfaces
  vector <double> out_channel;
  
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
