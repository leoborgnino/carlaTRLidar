/*-----------------------------------------------------------------------------
  -- Proyecto      : Logger
  -------------------------------------------------------------------------------
  -- Archivo       : Logger.h
  -- Organizacion  : Fundacion Fulgor
  -- Fecha         : 13 de Junio 2023
  -------------------------------------------------------------------------------
  -- Descripcion   : Logger Simple para guardar datos de Simulación
  -------------------------------------------------------------------------------
  -- Autor         : Leandro Borgnino
  -------------------------------------------------------------------------------
  -- Copyright (C) 2023 Fundacion Fulgor  All rights reserved
  -------------------------------------------------------------------------------
  -- $Id$
  -------------------------------------------------------------------------------*/

#ifndef Logger_H
#define Logger_H

// Includes common C++
#include <fstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h> 
#include <ctime>
#include <math.h>

// Includes Propios

using namespace std;

/*----------------------------------------------------------------------------*/
class Logger
{
  /*-----------------------------------------------------------------------*/
public:
  Logger();
  ~Logger();

  /**El metodo init, utiliza la clase loadSettings para determinar los valores de las
     variables que utiliza. */
  bool logVariable(string file_name, vector<double> data);

  /*-----------------------------------------------------------------------*/
private:
  void exposeVar();

  // Params
  

  // Variables

};
#endif
