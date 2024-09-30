// Time Resolved Lidar Fundación Fulgor
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include <chrono>
#include "Carla.h"
#include "Carla/Sensor/TimeResolvedLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "Materials/MaterialParameterCollectionInstance.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "JsonUtilities.h"


FActorDefinition ATimeResolvedLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast_time_resolved"));
}


ATimeResolvedLidar::ATimeResolvedLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);

  //Cargar el Reflectancemap desde un archivo json
  //const FString JsonMaterialsPath = FPaths::ProjectContentDir() + "/JsonFiles/materials.json";
  LoadReflectanceMapFromJson();

  //Cargar la lista de actores desde un archivo json 
  LoadVehiclesList();
}

void ATimeResolvedLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ATimeResolvedLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LidarData = FLidarData(Description.Channels);
  if (Description.LidarVFOVModel == 0)
    CreateLasersVFOV();
  else
    CreateLasersVFOV(GetVFOVPattern(Description.LidarVFOVModel));
  PointsPerChannel.resize(Description.Channels);
  
  if(Description.BeamDivergence)
    BDExtraPoints = Description.BD_nrings*(Description.BD_nrings+1)/2;
  else
    BDExtraPoints = 0;
  
  // Warning Remove and use API
  params.LAMBDA0 = Description.LAMBDA0;
  params.MAX_RANGE = Description.MAX_RANGE;
  params.DEBUG_GLOBAL = Description.DEBUG_GLOBAL;
  params.LOG_TX  = Description.LOG_TX;
  params.LOG_RX  = Description.LOG_RX;
  params.LOG_CHANNEL  = Description.LOG_CHANNEL;
  params.PTX  = Description.PTX;
  params.TAU_SIGNAL = Description.TAU_SIGNAL ;
  params.TX_FS = Description.TX_FS;
  params.TX_NOS = Description.TX_NOS;
  params.PULSE_SHAPE = Description.PULSE_SHAPE;
  params.F_BW = Description.F_BW;
  params.F_MIN = Description.F_MIN;
  params.PULSE_SHAPE = Description.PULSE_SHAPE;
  params.ARX = Description.ARX;
  params.CH_FS = Description.CH_FS;
  params.CH_NOS = Description.CH_NOS;
  params.PRX = Description.PRX;
  params.RPD = Description.RPD;
  params.RX_FS = Description.RX_FS;
  params.RX_NOS = Description.RX_NOS;
  
  
  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();

  // LiDAR Transceptor
  
  tx_lidar = new TxLidarPulsed();
  tx_lidar->init(&params);

  tx_lidar_fmcw = new TxLidarFMCW();
  tx_lidar_fmcw->init(&params);

  channel_lidar = new ChannelLidar();
  channel_lidar->init(&params);

  rx_lidar = new RxLidarPulsed();
  rx_lidar->init(&params);

  rx_lidar_fmcw = new RxLidarFMCW();
  rx_lidar_fmcw->init(&params);

}

void ATimeResolvedLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ATimeResolvedLidar::PostPhysTick);
  SimulateLidar(DeltaTime);

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto DataStream = GetDataStream(*this);
    DataStream.SerializeAndSend(*this, LidarData, DataStream.PopBufferFromPool());
  }
}

ATimeResolvedLidar::FDetection ATimeResolvedLidar::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf)
{
  auto start = std::chrono::high_resolution_clock::now();

  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);
  
  const float Distance = GetHitDistance(HitInfo,SensorTransf);

  //Atenuacion atmosferica en base a la distancia, por defecto de CARLA
  float CosAngle = 1.0;
  float Reflectance = 1.0;
  float AbsAtm = 1.0;

  AbsAtm = GetHitAtmAtt( Distance, Description.AtmospAttenRate, Description.ModelWeather );

  //Efecto del angulo del incidencia
  if (Description.INTENSITY_CALC)
    {
      CosAngle = GetHitCosIncAngle(HitInfo, SensorTransf);
      CosAngle = sqrtf(CosAngle);
      //Efecto de la reflectividad del material
      Reflectance = GetHitReflectance(HitInfo);
      //UE_LOG(LogCarla, Log, TEXT("CosAngle: %f"), CosAngle);
      //UE_LOG(LogCarla, Log, TEXT("Reflectance: %f"), Reflectance);
    }
    //La intensidad del punto tiene en cuenta:
    //Atenuacion atmosferica -> la intensidad sera menor a mayor distancia
    //Cos Ang Incidencia -> la intensidad mientras mas perpendicular a la superficie sea el rayo incidente
    //Reflectividad del material
  float IntensityNoiseStdDev = Description.NoiseStdDevIntensity;
  const float IntRec = (CosAngle * AbsAtm * Reflectance ) + RandomEngine->GetNormalDistribution(0.0f, IntensityNoiseStdDev);

  // Saturacion Intensidad
  if(IntRec <= 0.99 && IntRec > 0.0)
    Detection.intensity = IntRec;
  else if(IntRec > 0.99)
    Detection.intensity = 0.99;
  else
    Detection.intensity = 0.0;
      
  if (Description.TRANS_ON)
  {
    // LiDAR Transceptor
    vector<float> output_tx;
    if(Description.TransceptorArch == 0)
      output_tx = tx_lidar->run();
    else
      output_tx = tx_lidar_fmcw->run();
        
    // UE_LOG(LogCarla, Log, TEXT("TX: %d"), output_tx.size());

    vector<float> output_channel;
    output_channel = channel_lidar->run(output_tx,Distance,IntRec);

    // UE_LOG(LogCarla, Log, TEXT("CHANNEL: %d"), output_channel.size());
          
    vector<float> output_rx;
    if(Description.TransceptorArch == 0)
      output_rx = rx_lidar->run(output_tx,output_channel);
    else
      output_rx = rx_lidar_fmcw->run(output_tx,output_channel);
    Detection.time_signal = output_rx;
    //UE_LOG(LogCarla, Log, TEXT("RX: %d"), output_rx.size());
        
    // Calculo de la distancia
    auto it = max_element(output_rx.begin(),output_rx.end());
    int max_idx = distance(output_rx.begin(),it);
    double max_value = *it;
    double distance = ((max_idx+1-output_tx.size())/(params.RX_FS*params.RX_NOS))*LIGHT_SPEED/2.0;      // Calculo de la distancia

    FVector SensorLocation = SensorTransf.GetLocation(); 
    //Vector incidente, normalizado, entre sensor y punto de hit con el target
    FVector VectorIncidente = (HitPoint - SensorLocation).GetSafeNormal(); 
    auto vector_proc = (VectorIncidente*distance);
    Detection.point.x = vector_proc[0];
    Detection.point.y = vector_proc[1];
    Detection.point.z = vector_proc[2];

    //UE_LOG(LogCarla, Log, TEXT("Calculo distancia: %d %d %f %d %f %f"), max_idx, output_tx.size(), params.RX_FS, params.RX_NOS, LIGHT_SPEED, distance );
    //UE_LOG(LogCarla, Log, TEXT("Distancia: %f"), Distance);
    //UE_LOG(LogCarla, Log, TEXT("Distancia Transceptor: %f"), distance);

    // Only Debug
    // if(params.DEBUG_GLOBAL){
    //   if (params.LOG_RX){
    //     //cout << "Punto: " << Detection.point.x << " " << Detection.point.y << " " << Detection.point.z << endl;
    //     //cout << "Punto: " << vector_proc.X << " " << vector_proc.Y << " " << vector_proc.Z << endl;
    //     cout << output_tx.size() << " " << output_channel.size() << " " << endl;
    //     UE_LOG(LogCarla, Log, TEXT("Distancia: %f"), Distance);
    //     cout << "Distancia Receptor: " << distance << endl;
    //     //UE_LOG(LogCarla, Log, TEXT("Vector3: %s"), *(VectorIncidente*distance).ToString());
    //     //UE_LOG(LogCarla, Log, TEXT("Vector: %s"), *VectorIncidente.ToString());
        
    //     //UE_LOG(LogCarla, Log, TEXT("Vector2: %s"), *VectorIncidente_t.ToString());
        
    //     cout << "******************* Detección ***************" << endl;
    //           std::ostringstream oss;
    //     for (auto& i : Detection.time_signal){
    //       cout <<  i << " ";
    //             oss << i << ",";
    //           }
    //           oss << endl;
    //     cout << endl;
    //           std::string str = oss.str();
    //           FString unrealString(str.c_str());
    //           WriteFile("time_signal.txt",unrealString);
    //   }  
    //       }  

          
      }

    if(Description.DEBUG_GLOBAL)
      {
        using nano = std::chrono::nanoseconds;
        auto finish = std::chrono::high_resolution_clock::now();
        std::cout << "RAYCAST ELAPSED: "
      << std::chrono::duration_cast<nano>(finish - start).count()
      << " nanoseconds\n";
      }
  return Detection;
}

void ATimeResolvedLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
  Super::PreprocessRays(Channels, MaxPointsPerChannel);

  for (auto ch = 0u; ch < Channels; ch++) {
    for (auto p = 0u; p < MaxPointsPerChannel; p++) {
      RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
    }
  }
}

bool ATimeResolvedLidar::PostprocessDetection(FDetection& Detection) const
{
  const float Intensity = Detection.intensity;
  if(Intensity > Description.DropOffIntensityLimit)
    return true;
  else
    return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
}

void ATimeResolvedLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  {
    PointsPerChannel[idxChannel] = 0;
    for (auto n_hits = 0u; n_hits < RecordedHits[idxChannel].size(); ++n_hits )
      PointsPerChannel[idxChannel] += RecordedHits[idxChannel][n_hits].size();
  }

  LidarData.ResetMemory(PointsPerChannel);

  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
    for (auto& channel_hits : RecordedHits[idxChannel]) {
      for (auto& hit : channel_hits ){
        // UE_LOG(
	      //  LogCarla,
	      //  Warning,
	      //  TEXT("%s Distancia: %f \n Channel: %d \n Total Channels: %d \n Hits Per Channel: %d \n Hits per Hits: %d"),
	      //  *GetName(), hit.Distance, idxChannel, Description.Channels,  RecordedHits[idxChannel].size(), channel_hits.size());        
	      FDetection Detection = ComputeDetection(hit, SensorTransform);
        // UE_LOG(
	      // LogCarla,
	      // Warning,
	      // TEXT("%s: %f %f %f %f %f %f"),
	      // *GetName(), Detection.point.x, Detection.point.y, Detection.point.z, Detection.intensity, Detection.time_signal[0],Detection.time_signal[1]);
	      if (PostprocessDetection(Detection))
	        LidarData.WritePointSync(Detection);	    
	      else
	        PointsPerChannel[idxChannel]--;
      }
    }
  }
  LidarData.WriteChannelCount(PointsPerChannel);
}

void ATimeResolvedLidar::SimulateLidar(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ATimeResolvedLidar::SimulateLidar);
  const bool ModelMultipleReturn = Description.ModelMultipleReturn;
  const uint32 ChannelCount = Description.Channels;

  check(ChannelCount == LaserAngles.Num());

  const float CurrentHorizontalAngle = carla::geom::Math::ToDegrees(
								    SemanticLidarData.GetHorizontalAngle());
  const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov
    * DeltaTime;
  const float HorizontalFOVRes = Description.HorizontalFov * Description.RotationFrequency / Description.PointsPerSecond;
  const uint32_t PointsToScanWithOneLaser = AngleDistanceOfTick / HorizontalFOVRes;
  const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;
  
  if (PointsToScanWithOneLaser <= 0)
    {
      UE_LOG(
	     LogCarla,
	     Warning,
	     TEXT("%s: no points requested this frame, try increasing the number of points per second."),
	     *GetName());
      return;
    }


  //ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser*Description.NumReturnsMax*BDExtraPoints);
  RecordedHits.resize(ChannelCount);
  for (uint16_t i = 0; i < ChannelCount; ++i) {
    RecordedHits[i].resize(PointsToScanWithOneLaser);
    for (uint16_t j = 0; j < PointsToScanWithOneLaser; ++j) {
      RecordedHits[i][j].clear();
      RecordedHits[i][j].reserve(Description.NumReturnsMax*(1+BDExtraPoints) );
    }
  }
  
  PreprocessRays(ChannelCount, PointsToScanWithOneLaser*Description.NumReturnsMax*(1+BDExtraPoints));

  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(ChannelCount, [&](int32 idxChannel)
     {
       TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);
       
       FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
       TraceParams.bTraceComplex = true;
       TraceParams.bReturnPhysicalMaterial = false;
       
       for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) 
       {
        
	      const float VertAngle = LaserAngles[idxChannel];
	      const float HorizAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfLaserMeasure* idxPtsOneLaser, Description.HorizontalFov) - Description.HorizontalFov / 2;
	      const bool PreprocessResult = RayPreprocessCondition[idxChannel][idxPtsOneLaser];

        FHitResult HitResult;
	      TArray<FHitResult> HitsResult;
	      for (int ii = 0; ii < 1 + BDExtraPoints; ii++)
	        if (PreprocessResult && ShootLaser(VertAngle+getBD_VAngle(ii), HorizAngle+getBD_HAngle(ii), HitResult, TraceParams, idxChannel, ModelMultipleReturn))
          //if (PreprocessResult && ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams, idxChannel, ModelMultipleReturn))
          {
	          uint16_t cnt_hit = 0;
	          //for (auto& hitInfo : HitsResult)
            //{
              // UE_LOG(
	            //  LogCarla,
	            //  Warning,
	            //  TEXT("%s Distancia: %f %d %f %f"),
	            //  *GetName(), HitResult.Distance, cnt_hit, VertAngle+getBD_VAngle(ii), HorizAngle+getBD_HAngle(ii));
	            if( !ModelMultipleReturn || cnt_hit < Description.NumReturnsMax ) 
		            RecordedHits[idxChannel][ii+cnt_hit].emplace_back(HitResult);
              cnt_hit++;
            //} 
	        }
      }
    });
  }
  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  FTransform ActorTransf = GetTransform();
  ComputeAndSaveDetections(ActorTransf);

  const float HorizontalAngle = carla::geom::Math::ToRadians(
							     std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov));
  SemanticLidarData.SetHorizontalAngle(HorizontalAngle);
}


bool ATimeResolvedLidar::ShootLaser(const float VerticalAngle, const float HorizontalAngle, FHitResult& HitResult, FCollisionQueryParams& TraceParams, int32 idxChannel,const bool ModelMultipleReturn) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

  FHitResult HitInfo(ForceInit);

  FTransform ActorTransf = GetTransform();
  FVector LidarBodyLoc = ActorTransf.GetLocation();
  FRotator LidarBodyRot = ActorTransf.Rotator();

  FRotator LaserRot (VerticalAngle, HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
  FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
    LaserRot,
    LidarBodyRot
  );

  const auto Range = Description.Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  GetWorld()->ParallelLineTraceSingleByChannel(
    HitInfo,
    LidarBodyLoc,
    EndTrace,
    ECC_GameTraceChannel2,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam
  );

  if (HitInfo.bBlockingHit) {
    HitResult = HitInfo;
    return true;
  } else {
    return false;
  }
}

bool ATimeResolvedLidar::ShootLaser(const float VerticalAngle, const float HorizontalAngle, TArray<FHitResult>& HitResults, FCollisionQueryParams& TraceParams, int32 idxChannel, const bool MultiShoot)
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

  FHitResult HitInfo(ForceInit);

  FTransform ActorTransf = GetTransform();
  FVector LidarBodyLoc = ActorTransf.GetLocation();
  FRotator LidarBodyRot = ActorTransf.Rotator();

  FRotator LaserRot (VerticalAngle, HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
  FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
							   LaserRot,
							   LidarBodyRot
							   );

  const auto Range = Description.Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  //Calcular la posicion del disparo segun el canal
  FVector ShootLoc = GetShootLoc(LidarBodyLoc, ResultRot, idxChannel);

  //CAMBIOS DE MODELO  
  //El Trace debe ser complejo y retornar el face index para obtener el material
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnFaceIndex = true;

  // Only Debug Time Trace
  double TimeStampStart;
  if(Description.DEBUG_GLOBAL)
    TimeStampStart = FPlatformTime::Seconds() * 1000.0;

  // Shoot Laser
  if (MultiShoot)
    GetWorld()->LineTraceMultiByChannel( // Parallel ?
					HitResults,
					ShootLoc,
					EndTrace,
					ECC_GameTraceChannel2,
					TraceParams,
					FCollisionResponseParams::DefaultResponseParam
					 );
  else
    {
      HitResults.Add(HitInfo);
      GetWorld()->ParallelLineTraceSingleByChannel(
						   HitResults[0],
						   ShootLoc,
						   EndTrace,
						   ECC_GameTraceChannel2,
						   TraceParams,
						   FCollisionResponseParams::DefaultResponseParam
						   );
    }

  double TimeStampEnd;
  double TimeTrace;
  // Only Debug Time Trace
  if(Description.DEBUG_GLOBAL)
    {
      TimeStampEnd = FPlatformTime::Seconds() * 1000.0;
      TimeTrace = TimeStampEnd - TimeStampStart;
    }


  bool state_hits = true;
  
  // Get Distances from Hits
  /*
  TArray<float> DistanceTraces;
  */
  uint32_t cnt_hit = 0;
  for (const auto& hitInfo : HitResults) 
    { 
      float DistanceTrace = GetHitDistance(hitInfo,ActorTransf);
      //DistanceTraces.Add(DistanceTrace);
      // UE_LOG(
      // LogCarla,
	    // Warning,
	    // TEXT("%s Distancia: %f %d"),
	    // *GetName(),DistanceTrace, cnt_hit);
      cnt_hit++;
    }

    /*
      if(Description.DEBUG_GLOBAL)
	{
	  // Log Time/Distance
	  //nombre del archivo para log
	  FString NameLogFile = TEXT("Log_channel_") + FString::FromInt(idxChannel) + TEXT(".txt");
	  //tiempo del disparo para log
	  FString TimeLog = TEXT("Tiempo:") +FString::SanitizeFloat(TimeTrace);
	  //ditancia del disparo para log
	  FString DistLog = TEXT("Distancia:") +FString::SanitizeFloat(DistanceTrace);
	  WriteFile(NameLogFile,DistLog);
	  WriteFile(NameLogFile,TimeLog);
	}
  }
  */

  // Blocking Removed: I think I don't care in multiple returns
  //eliminar puntos que son del vehiculo recolector de datos WARNING
  //if (HitResults[0].bBlockingHit )
  //{
  if (!(Description.TRANS_ON))
    if(UnderMinimumReturnDistance(HitResults[0],ActorTransf))
      state_hits = false;
  if (!(Description.TRANS_ON) && (Description.INTENSITY_CALC) )
    {
    //determinar si el punto corresponde a una reflectancia detectable
    if(!CheckDetectableReflectance(HitResults[0],ActorTransf))
    state_hits = false;
     }
  //}
  //else
  //  state_hits = false;
   //
   //{
   //}


  return state_hits;
}


float ATimeResolvedLidar::GetHitDistance(const FHitResult& HitInfo,const FTransform& SensorTransf) const
{
  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  float Distance = Detection.point.Length();

  return Distance;
}

float ATimeResolvedLidar::GetHitAtmAtt(const float Distance, const float AttenAtmRate,const int mode) const
{
  if(mode == 0)
    return exp(-AttenAtmRate * Distance);
  else
    return exp(-AttenAtmRate * Distance); // TODO Statistics Weather
}

float ATimeResolvedLidar::GetHitCosIncAngle(const FHitResult& HitInfo, const FTransform& SensorTransf) const{

  const FVector HitPoint = HitInfo.ImpactPoint;
  //Posicion del sensor
  FVector SensorLocation = SensorTransf.GetLocation(); 
  //Vector incidente, normalizado, entre sensor y punto de hit con el target
  FVector VectorIncidente = - (HitPoint - SensorLocation).GetSafeNormal(); 
  //Vector normal a la superficie de hit, normalizado
  FVector VectorNormal = HitInfo.ImpactNormal;
  //Producto punto entre ambos vector, se obtiene el coseno del ang de incidencia
  float CosAngle = FVector::DotProduct(VectorIncidente, VectorNormal);
  //CosAngle = sqrtf(CosAngle);
  return CosAngle;
}


//REMOVE
// Función para escribir en el archivo si el string es nuevo
void ATimeResolvedLidar::WriteIfUnique(const std::string& newString, const std::string& filePath, std::set<std::string>& uniqueStrings)
{
    // Verificar si el string ya está en la lista
    if (uniqueStrings.find(newString) == uniqueStrings.end())
    {
        // Si no está, añadirlo al set y escribirlo en el archivo
        uniqueStrings.insert(newString);
        UE_LOG(LogCarla, Warning, TEXT("Material: %s"), *FString(newString.c_str()));
    }
}
// REMOVE

float ATimeResolvedLidar::GetHitReflectance( const FHitResult& HitInfo )
{
  AActor* ActorHit = HitInfo.GetActor();
  FString ActorHitName = ActorHit->GetName();
  
  float Reflectance = 1.0;

  FString MaterialNameHit = GetHitMaterialName(HitInfo);

  UPrimitiveComponent* HitComponent = HitInfo.Component.Get();
    if (HitComponent)
    {
        UMaterialInterface* HitMaterial = HitComponent->GetMaterial(0);  // Índice del material (0 para el primer material)
        if (HitMaterial)
        {
            FString MaterialName = HitMaterial->GetName();
            std::string filePath = "D:/CARLA/lidar-systems-level/CARLA_scripts/output_materials.txt";
            std::string StandardString = TCHAR_TO_UTF8(*MaterialName);
            WriteIfUnique(StandardString, filePath, uniqueStrings);
            UE_LOG(LogCarla, Warning, TEXT("Material: %s"), *MaterialName);
        }
        else
        {
            //UE_LOG(LogCarla, Log, TEXT("Material: None"));
        }
    }
  
  // UE_LOG(
  // LogCarla,
	// Warning,
	// TEXT("%s Material: %s %s"),
	// *GetName(),*MaterialNameHit, HitInfo.Component->CustomDepthStencilValue);


  
  //Segun si el nombre del actor, corresponde a un actor al cual computar su material
  bool CriticalVehicle = IsCriticalVehicle(ActorHitName);
  if(CriticalVehicle)
    {
      //Se obtiene el nombre del material del hit
      FString MaterialNameHit = GetHitMaterialName(HitInfo);
      //Si el actor corresponde a un ciclista y no se obtiene material, coresponde a la parte de la persona
      if(IsCyclist(ActorHitName) && (MaterialNameHit.Compare("NoMaterial") == 0))
	Reflectance = GetMaterialReflectanceValue(TEXT("Pedestrian"));
      else
	Reflectance = GetMaterialReflectanceValue(MaterialNameHit);
    }
  else if(IsPedestrian(ActorHitName))
    Reflectance = GetMaterialReflectanceValue(TEXT("Pedestrian"));
  else
    //Se le asigna una reflectivdad por defeto a los materiales no criticos
    Reflectance = GetMaterialReflectanceValue(TEXT("NoMaterial"));

  return Reflectance;
}

float ATimeResolvedLidar::GetMaterialReflectanceValue(FString MaterialNameHit)const {

  const double* ReflectancePointer;
  bool MaterialFound = false;
  float Reflectance = 1.0;
  //Se recorre la lista de materiales con su respectiva reflectividad
  for (auto& Elem : ReflectanceMap)
    {
      FString MaterialKey = Elem.Key;
      //comprueba de si el nombre del material esta incluido en el material del hit
      if(MaterialNameHit.Contains(MaterialKey)){
        //cuando se encuentra, se obtiene el valor de reflectividad asociado a ese material
        Reflectance = (float)Elem.Value;
        MaterialFound=true;
        //WriteFile(MaterialNameHit);
        break;
      }
    }

  if(!MaterialFound){
    //Se le asigna una reflectivdad por defeto a los materiales no criticos
    ReflectancePointer = ReflectanceMap.Find(TEXT("NoMaterial"));
    Reflectance = (float)*ReflectancePointer;
  }

  return Reflectance;
}

FString ATimeResolvedLidar::GetHitMaterialName(const FHitResult& HitInfo) const{

  UPrimitiveComponent* ComponentHit = HitInfo.GetComponent();
    
  if(ComponentHit){
    if (HitInfo.FaceIndex != -1) {
      int32 section = 0;
      UMaterialInterface* MaterialIntHit = ComponentHit->GetMaterialFromCollisionFaceIndex(HitInfo.FaceIndex, section);
      return MaterialIntHit->GetName();

    }
  }

  return FString(TEXT("NoMaterial"));
    
}


//Funcion implementada para leer desde un json, la reflectividad asociada a cada material
//y cargarlo en el ReflectanceMap
void ATimeResolvedLidar::LoadReflectanceMapFromJson(){

  //path del archivo json
  const FString FilePath = FPaths::ProjectDir() + "../../../CARLA_scripts/LidarModelFiles/materials.json";
  
  //const FString JsonFilePath = FPaths::ProjectContentDir() + "/JsonFiles/materials.json";

  //carga el json a un string
  FString JsonString;
  FFileHelper::LoadFileToString(JsonString,*FilePath);

  TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject());
  TSharedRef<TJsonReader<>> JsonReader = TJsonReaderFactory<>::Create(JsonString);

  //parsea el string a un jsonobject
  if (FJsonSerializer::Deserialize(JsonReader, JsonObject) && JsonObject.IsValid())
    { 
      //obtener el array de materials
      TArray<TSharedPtr<FJsonValue>> objArray=JsonObject->GetArrayField("materials");
      
      //iterar sobre todos los elmentos del array
      for(int32 index=0;index<objArray.Num();index++)
	{
	  TSharedPtr<FJsonObject> obj = objArray[index]->AsObject();
	  if(obj.IsValid()){
          
	    //de cada elemento, obtener nombre y reflectivity
	    FString name = obj->GetStringField("name");
	    double reflec = obj->GetNumberField("reflectivity");

	    //cargar en el ReflectivityMap
	    ReflectanceMap.Add(name,reflec);

	    GLog->Log("name:" + name);
	    GLog->Log("reflectivity:" + FString::SanitizeFloat(reflec));
	  }
	}
    }
}


void ATimeResolvedLidar::LoadVehiclesList(){

  //path del archivo json
  const FString FilePath = FPaths::ProjectDir() + "../../../CARLA_scripts/LidarModelFiles/vehicles.json";
  
  //const FString JsonFilePath = FPaths::ProjectContentDir() + "/JsonFiles/materials.json";

  //carga el json a un string
  FString JsonString;
  FFileHelper::LoadFileToString(JsonString,*FilePath);

  TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject());
  TSharedRef<TJsonReader<>> JsonReader = TJsonReaderFactory<>::Create(JsonString);

  //parsea el string a un jsonobject
  if (FJsonSerializer::Deserialize(JsonReader, JsonObject) && JsonObject.IsValid())
    { 
      //obtener el array de materials
      TArray<TSharedPtr<FJsonValue>> objArray=JsonObject->GetArrayField("vehicles");
      
      //iterar sobre todos los elmentos del array
      for(int32 index=0;index<objArray.Num();index++)
	{
	  TSharedPtr<FJsonObject> obj = objArray[index]->AsObject();
	  if(obj.IsValid()){
          
	    //de cada elemento, obtener el nombre del actor
	    FString name = obj->GetStringField("unreal_actor_name");

	    VehiclesList.Add(name);

	    GLog->Log("name:" + name);

	  }
	}
    }

}

bool ATimeResolvedLidar::WriteFile(FString Filename, FString String) const {
  const FString FilePath = FPaths::ProjectContentDir() + TEXT("/LogFiles/") + Filename;

  FString new_String = FString::Printf( TEXT( "%s \n" ), *String);
  FFileHelper::SaveStringToFile(new_String, *FilePath,
				FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);

  return true;
}

bool ATimeResolvedLidar::IsCriticalVehicle(FString ActorHitName) const
{
  bool ActorFound = false;
  //Determinar si el actor del hit, esta dentro de los vevhiculos a los cuales computar los materiales
  for (int32 i=0; i!=VehiclesList.Num();i++)
    if(ActorHitName.Contains(VehiclesList[i]))
      {
	ActorFound=true;
	break;
      }
  return ActorFound;
}

bool ATimeResolvedLidar::IsPedestrian(FString ActorHitName) const
{
  return ActorHitName.Contains(TEXT("Walker"));
}

bool ATimeResolvedLidar::IsCyclist(FString ActorHitName) const
{
  return ActorHitName.Contains(TEXT("Bike"));
}

bool ATimeResolvedLidar::CheckDetectableReflectance(const FHitResult& HitInfo,const FTransform& SensorTransf){
    
  const bool ModelReflectanceLimitsFunction = Description.ModelReflectanceLimitsFunction;

  if(ModelReflectanceLimitsFunction){
    float Distance = GetHitDistance(HitInfo,SensorTransf);
    const float Reflectance = GetMaterialReflectanceValue(GetHitMaterialName(HitInfo));

    //Funcion de rango de deteccion segun reflec R(d) = a + b.d^2
    //float a = 0.0005f;
    float a = Description.ReflectanceLimitsFunctionCoeffA;
    //float b = 0.000054f;
    float b = Description.ReflectanceLimitsFunctionCoeffB;

    float ReflectanceLimit = a + b * (Distance*Distance);

    if(Reflectance >= ReflectanceLimit){
      return true;
    }else{
      float dif = ReflectanceLimit - Reflectance;
      float RangeRandom = 0.5 * ReflectanceLimit; //ancho del rango de reflectancia por debajo del umbral, donde el comportamiento es aleatorio
      if(RangeRandom > 0.15){
	RangeRandom = 0.15;}
      return RandomEngine->GetUniformFloat() > (dif/RangeRandom); //si da true, el punto se cuenta, mientras mas grande el dif, menos chances de contar el punto
    }
      
  }else{

    return true;
  }
    
}
  
bool ATimeResolvedLidar::UnderMinimumReturnDistance(const FHitResult& HitInfo,const FTransform& SensorTransf){
  //descartar puntos que estan por debajo de la minima
  float Distance = GetHitDistance(HitInfo,SensorTransf);
  float MinimumReturnDistance = 0.5;

  return (Distance <= MinimumReturnDistance);
}

FVector ATimeResolvedLidar::GetShootLoc(FVector LidarBodyLoc, FRotator ResultRot, int32 idxChannel){
  //Calcular el punto de disparo de los laser segun el canal
    
  if(Description.ModelHDL64LasersGroups){
    //HDL64 divide los 64 lasers en 2 bloques (upper y lower), con 2 grupos(left y right).

    float VerticalDistance = 2.5;//entre bloques, verticalmente hay 5 cm de distancia, desde el centro seria la mitad
      
    FVector UpTrans= FVector(0.0,0.0,VerticalDistance);
    FVector DownTrans= FVector(0.0,0.0,-1.0*VerticalDistance);

    //Ubicacion del centro de los bloques upper y lower
    FVector UpperBlockLoc = UpTrans + LidarBodyLoc;
    FVector LowerBlockLoc = DownTrans + LidarBodyLoc;

    //Para determinar la posicion de los grupo left y right, se tiene en cuenta la orientacion del sensor
    //y se obtiene el rightVector y leftVector de esa orientacion.

    float HorizontalDistance = 2.5; //entre lentes, horizontalmente hay 5 cm de distancia, desde el centro seria la mitad
    FVector RigthGroupTrans = HorizontalDistance * UKismetMathLibrary::GetRightVector(ResultRot);
    FVector LeftGroupTrans = HorizontalDistance * -1.0 * UKismetMathLibrary::GetRightVector(ResultRot);

    //Ubicacion de cada grupo, desplazando la ubicacion del centro de cada bloque, a la izquierda o derecha
    FVector UpperRigthGroupLoc = UpperBlockLoc + RigthGroupTrans;
    FVector UpperLeftGroupLoc = UpperBlockLoc + LeftGroupTrans;
    FVector LowerRigthGroupLoc = LowerBlockLoc + RigthGroupTrans;
    FVector LowerLeftGroupLoc = LowerBlockLoc + LeftGroupTrans;

    int32 GroupOfLaser = GetGroupOfChannel(idxChannel);
    //0: UpperLeft
    //1: UpperRigth
    //2: LowerLeft
    //3: LowerRigth

    switch(GroupOfLaser){
    case 0:
      return UpperLeftGroupLoc;
    case 1:
      return UpperRigthGroupLoc;
    case 2:
      return LowerLeftGroupLoc;
    case 3:
      return LowerRigthGroupLoc;
    }
  }
      
  return LidarBodyLoc;
}

int32 ATimeResolvedLidar::GetGroupOfChannel(int32 idxChannel){
  //Determinar a que grupo corresponde cada canal para el HDL64
  //Segun el manual:
  //0 a 31: upper block, pares left, impares rigth
  //32 a 63: lower block, pares left, impares rigth
  //Se asigna un numero a cada grupo: 
  //0: UpperLeft
  //1: UpperRigth
  //2: LowerLeft
  //3: LowerRigth

  if(idxChannel < 32){
    if(idxChannel%2 == 0){
      return 0;
    }else{
      return 1;
    }
  }else{
    if(idxChannel%2 == 0){
      return 2;
    }else{
      return 3;
    }
  }

}

// Beam Divergence System

float ATimeResolvedLidar::get_SubrayRing(int n_subray)
{
  int subray_ring = 0;
  for (int ii=1; ii < Description.BD_nrings+1; ii++)
    if (n_subray < int(2*M_PI*ii))
      subray_ring = ii;
  return subray_ring;
}


float ATimeResolvedLidar::getBD_HAngle(int n_subray)
{
  if(Description.BeamDivergence)
  {
    int const CurrentRing = get_SubrayRing(n_subray);
    float const TotalSubraysRing = int(2*M_PI*CurrentRing);
    float AngleStepRing = ( (2 * M_PI) / TotalSubraysRing);
    
    return ( Description.BD_hrad * n_subray * cos(AngleStepRing) );
  }
  else
    return 0;
}


float ATimeResolvedLidar::getBD_VAngle(int n_subray)
{
  if(Description.BeamDivergence)
  {
    int const CurrentRing = get_SubrayRing(n_subray);
    float const TotalSubraysRing = int(2*M_PI*CurrentRing);
    float AngleStepRing = ( (2 * M_PI) / TotalSubraysRing);
    
    return ( Description.BD_hrad * n_subray * sin(AngleStepRing) );
  }
  else
    return 0;
}


// VFOV

void ATimeResolvedLidar::CreateLasersVFOV()
{
  const auto NumberOfLasers = Description.Channels;
  check(NumberOfLasers > 0u);
  const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
    (Description.UpperFovLimit - Description.LowerFovLimit) /
    static_cast<float>(NumberOfLasers - 1);
  LaserAngles.Empty(NumberOfLasers);
  for(auto i = 0u; i < NumberOfLasers; ++i)
  {
    const float VerticalAngle =
        Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
    LaserAngles.Emplace(VerticalAngle);
  }
}

void ATimeResolvedLidar::CreateLasersVFOV(std::vector<float> angles)
{
  check(Description.Channels > 0u);
  LaserAngles.Empty(Description.Channels);
  for(uint32 i = 0u; i < Description.Channels; ++i)
  {
    const float VerticalAngle = angles[i];
    LaserAngles.Emplace(VerticalAngle);
  }
}

std::vector<float> ATimeResolvedLidar::GetVFOVPattern(int LiDARName)
{
  if (LiDARName == 0)// "pandas64")
  return {		14.882, 11.032, 8.059,   5.057,   3.04,    2.028,  1.86,    1.688,
			1.522,  1.351,  1.184,   1.013,   -1.184,  -1.351, -1.522,  -1.688,
			-1.86,  -2.028, -2.198,  -2.365,  -2.536,  -2.7,   -2.873,  0.846,
			0.675,  0.508,  0.337,   0.169,   0,       -0.169, -0.337,  -0.508,
			-0.675, -0.845, -1.013,  -3.04,   -3.21,   -3.375, -3.548,  -3.712,
			-3.884, -4.05,  -4.221,  -4.385,  -4.558,  -4.72,  -4.892,  -5.057,
			-5.229, -5.391, -5.565,  -5.726,  -5.898,  -6.061, -7.063,  -8.059,
			-9.06,  -9.885, -11.032, -12.006, -12.974, -13.93, -18.889, -24.897 };
  else if (LiDARName == 1)// "ruby128")
    return {
			-13.565, -1.09,   -4.39, 1.91,  -6.65,   -0.29,  -3.59, 2.71,  -5.79,
			0.51,    -2.79,   3.51,  -4.99, 1.31,    -1.99,  5.06,  -4.19, 2.11,
			-19.582, -1.29,   -3.39, 2.91,  -7.15,   -0.49,  -2.59, 3.71,  -5.99,
			0.31,    -1.79,   5.96,  -5.19, 1.11,    -0.99,  -4.29, 2.01,  -25,
			-0.19,   -3.49,   2.81,  -7.65, 0.61,    -2.69,  3.61,  -6.09, 1.41,
			-1.89,   5.46,    -5.29, 2.21,  -16.042, -1.19,  -4.49, 3.01,  -6.85,
			-0.39,   -3.69,   3.81,  -5.89, 0.41,    -2.89,  6.56,  -5.09, 1.21,
			-2.09,   -8.352,  -0.69, -3.99, 2.31,    -6.19,  0.11,  -3.19, 3.11,
			-5.39,   0.91,    -2.39, 3.96,  -4.59,   1.71,   -1.59, 7.41,  -3.79,
			2.51,    -10.346, -0.89, -2.99, 3.31,    -6.39,  -0.09, -2.19, 4.41,
			-5.59,   0.71,    -1.39, 11.5,  -4.79,   1.51,   -0.59, -3.89, 2.41,
			-11.742, 0.21,    -3.09, 3.21,  -6.5,    1.01,   -2.29, 4.16,  -5.69,
			1.81,    -1.49,   9,     -4.89, 2.61,    -9.244, -0.79, -4.09, 3.41,
			-6.29,   0.01,    -3.29, 4.71,  -5.49,   0.81,   -2.49, 15,    -4.69,
			1.61,    -1.69 };
  else if (LiDARName == 2)//"pandar128")
    {
      std::vector<float> pandar128;
      for (int i = 0; i < 64; i++) {
		pandar128.push_back(-6 + 0.125 * i);
	}
	for (int i = 0; i < 36; i++) {
		pandar128.push_back(-6.5 - 0.5 * i);
	}
	for (int i = 0; i < 24; i++) {
		pandar128.push_back(2 + 0.5 * i);
	}
	for (int i = 0; i < 2; i++) {
		pandar128.push_back(14 + i);
		pandar128.push_back(-26 + i);
	}
	return pandar128;
    }
  else if (LiDARName == 3) //"pandar40m")
    return {
	    15,    11,    8,     5,     3,     2,     1.67,  1.33,  1,     0.67,
	    0.33,  0,     -0.33, -0.67, -1,    -1.33, -1.67, -2.00, -2.33, -2.67,
	    -3.00, -3.33, -3.67, -4.00, -4.33, -4.67, -5.00, -5.33, -5.67, -6.00,
	    -7,    -8,    -9,    -10,   -11,   -12,   -13,   -14,   -19,   -25 };
  else //if (LiDARName == "hdl64")
    {
      std::vector<float> hdl64; 
      for (int i = 0; i < 64; i++) {
        hdl64.push_back(-24.9 + 0.427 * i);
      }
      return hdl64;
    }
}
