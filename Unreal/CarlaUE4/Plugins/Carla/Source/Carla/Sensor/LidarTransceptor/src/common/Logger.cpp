/*-----------------------------------------------------------------------------
  -- Proyecto      : Logger
  -------------------------------------------------------------------------------
  -- Archivo       : Logger.cpp
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

#include "Logger.h"

/*----------------------------------------------------------------------------*/
Logger::Logger()
{
    
  printf("Logger :: Has been created\n");

};

bool Logger::logVariable(string file_name, vector<double> data)
{
  ofstream file_log;
  
  file_log.open(file_name);
  for (long unsigned int ii = 0; ii < data.size(); ii++)
    file_log << data[ii] << " ";
  file_log << endl;
  file_log.close();
    
  return true; // Warning: Agregar gestión de errores
  
}
