// Time Resolved Lidar Fundación Fulgor
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Sensor/RayCastSemanticLidar.h"
// Tx
#include "Carla/Sensor/LidarTransceptor/src/common/constants.h"
#include "Carla/Sensor/LidarTransceptor/src/tx_lidar/TxLidarPulsed.h"
#include "Carla/Sensor/LidarTransceptor/src/tx_lidar/TxLidarFMCW.h"
// Channel
#include "Carla/Sensor/LidarTransceptor/src/channel_lidar/ChannelLidar.h"
// Rx
#include "Carla/Sensor/LidarTransceptor/src/rx_lidar/RxLidarPulsed.h"
#include "Carla/Sensor/LidarTransceptor/src/rx_lidar/RxLidarFMCW.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/LidarData.h>
#include <compiler/enable-ue4-macros.h>
#include "Runtime/Core/Public/Async/ParallelFor.h"

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

  void PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) override;
  void ComputeAndSaveDetections(const FTransform& SensorTransform) override;

  // ShootLaser Override
  bool ShootLaser(const float VerticalAngle, const float HorizontalAngle, TArray<FHitResult>& HitResults, FCollisionQueryParams& TraceParams, int32 idxChannel, const bool MultiShoot);
  void SimulateLidar(const float DeltaTime);

  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime);

private:
  /// Compute the received intensity of the point
  float ComputeIntensity(const FSemanticDetection& RawDetection) const;
  FDetection ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const;

  bool PostprocessDetection(FDetection& Detection) const;

  bool WriteFile(FString Filename, FString String) const;

  bool IsCriticalVehicle(FString ActorHitName) const;
  bool IsPedestrian(FString ActorHitName) const;
  bool IsCyclist(FString ActorHitName) const;

  FLidarData LidarData;
  vector<vector<vector<FHitResult>>> RecordedHits; // 3D for multiple detections

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
  TMap<FString, double> ReflectanceMap;

  //Funcion para leer un archivo json y cargar el reflectivity map
  void LoadReflectanceMapFromJson(); 

  //Lista de los nombres de vehiculos, para los cuales se van a tener en cuenta los materiales
  //Se lo inicializa leyendo desde un archivo json en el constructor de la clase
  TArray<FString> VehiclesList;

  //Cargar la lista de vehiculos desde archivo json
  void LoadVehiclesList();
  
  FString GetHitMaterialName(const FHitResult& HitInfo) const;
  float GetHitCosIncAngle(const FHitResult& HitInfo, const FTransform& SensorTransf) const;
  float GetHitReflectance( const FHitResult& HitInfo ) const;
  float GetMaterialReflectanceValue(FString MaterialName) const;
  float GetHitDistance(const FHitResult& HitInfo,const FTransform& SensorTransf) const;
  float GetHitAtmAtt(const float Distance, const float AttenAtmRate,const int mode) const;

  //Determinar si el hit es valido, segun la refletividad y la funcion de rango
  bool CheckDetectableReflectance(const FHitResult& HitInfo,const FTransform& SensorTransf);
  bool UnderMinimumReturnDistance(const FHitResult& HitInfo,const FTransform& SensorTransf);

  // Emisor Disposición
  FVector GetShootLoc(FVector LidarBodyLoc, FRotator ResultRot, int32 idxChannel);
  int32 GetGroupOfChannel(int32 idxChannel);

  // Beam Divergence
  int BDExtraPoints;
  
  float get_SubrayRing(int n_subray);
  float getBD_HAngle(int n_subray);
  float getBD_VAngle(int n_subray);
  
  // Transceptor LiDAR
  parametersLiDAR params;
 
  TxLidarPulsed * tx_lidar;
  TxLidarFMCW * tx_lidar_fmcw;
  ChannelLidar *  channel_lidar;
  RxLidarPulsed * rx_lidar;
  RxLidarFMCW * rx_lidar_fmcw;

};
