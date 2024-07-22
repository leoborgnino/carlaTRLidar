#ifndef CONSTANTS_H
#define CONSTANTS_H

const double LIGHT_SPEED = 3e8;           // Velocidad de la Luz [m/s]
const double Q_ELECT = 1.6e-19;           // Carga Electrica [C]
//const double PI = 3.14159265358979323846; // PI [rad]

struct parametersLiDAR {
  // Global
  double LAMBDA0;
  double MAX_RANGE;
  bool DEBUG_GLOBAL;
  bool LOG_TX;
  bool LOG_RX;
  bool LOG_CHANNEL;

  // TX
  double PTX;         // Potencia del Transmisor [Watts]
  double TAU_SIGNAL;  // Duración del pulso [s] si lo aumento, gano SNR, pierdo resolución
  double TX_FS;          // Frecuencia de Muestreo [Hz]
  unsigned int TX_NOS;   // Sobremuestreo [Veces]

  // Channel
  
  double       ARX;     // Ganancia de la óptica del receptor (pi*(2.54e-2/2)^2) diametro 1 pulgada del receptor
  double       CH_FS;   // Frecuencia de Muestreo [Hz]
  unsigned int CH_NOS;  // Sobremuestreo [Veces]

    // RX
  double PRX;          // Amplificador del detector
  double RPD;          // Sensibilidad del fotodetector [A/W] 
  double RX_FS;        // Frecuencia de Muestreo [Hz]
  unsigned int RX_NOS; // Sobremuestreo [Veces]
  
};
 
#endif
