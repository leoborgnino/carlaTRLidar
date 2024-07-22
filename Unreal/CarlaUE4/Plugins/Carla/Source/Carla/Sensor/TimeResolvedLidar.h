// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <vector>

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Sensor/RayCastSemanticLidar.h"
// Tx
#include "Carla/Sensor/LidarTransceptor/src/common/constants.h"
#include "Carla/Sensor/LidarTransceptor/src/tx_lidar/TxLidarPulsed.h"
// Channel
#include "Carla/Sensor/LidarTransceptor/src/channel_lidar/ChannelLidar.h"
// Rx
#include "Carla/Sensor/LidarTransceptor/src/rx_lidar/RxLidarPulsed.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/LidarData.h>
#include <compiler/enable-ue4-macros.h>

#include "TimeResolvedLidar.generated.h"


/// A ray-cast based Lidar sensor.
UCLASS()
class CARLA_API ATimeResolvedLidar : public ARayCastSemanticLidar
{
  GENERATED_BODY()

  using FLidarData = carla::sensor::data::LidarData;
  using FDetection = carla::sensor::data::LidarDetection;

public:
  static FActorDefinition GetSensorDefinition();

  ATimeResolvedLidar(const FObjectInitializer &ObjectInitializer);
  virtual void Set(const FActorDescription &Description) override;
  virtual void Set(const FLidarDescription &LidarDescription) override;

  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime);

private:
  /// Compute the received intensity of the point
  float ComputeIntensity(const FSemanticDetection& RawDetection) const;
  FDetection ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const;

  void PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) override;
  bool PostprocessDetection(FDetection& Detection) const;

  void ComputeAndSaveDetections(const FTransform& SensorTransform) override;
  bool WriteFile(FString Filename, FString String);

  FLidarData LidarData;

  /// Enable/Disable general dropoff of lidar points
  bool DropOffGenActive;

  /// Slope for the intensity dropoff of lidar points, it is calculated
  /// throught the dropoff limit and the dropoff at zero intensity
  /// The points is kept with a probality alpha*Intensity + beta where
  /// alpha = (1 - dropoff_zero_intensity) / droppoff_limit
  /// beta = (1 - dropoff_zero_intensity)
  float DropOffAlpha;
  float DropOffBeta;

  //Map de materialName,reflectivity para todos los materiales 
  //Se lo inicializa leyendo desde un archivo json en el constructor de la clase
  TMap<FString, double> ReflectivityMap;

  //Funcion para leer un archivo json y cargar el reflectivity map
  void LoadReflectivityMapFromJson(); 

  //Lista de los nombres de actores, para los cuales se van a tener en cuenta los materiales
  //Se lo inicializa leyendo desde un archivo json en el constructor de la clase
  TArray<FString> ActorsList;
  void LoadActorsList();
  
  FString GetHitMaterialName(const FHitResult& HitInfo) const;
  float GetHitMaterialSpecular(const FHitResult& HitInfo) const;


  // Transceptor LiDAR
  parametersLiDAR params;

  TxLidarPulsed * tx_lidar;
  ChannelLidar *  channel_lidar;
  RxLidarPulsed * rx_lidar;

};