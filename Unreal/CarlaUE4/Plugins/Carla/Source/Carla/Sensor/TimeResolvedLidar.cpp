// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
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

  //Cargar el reflectivitymap desde un archivo json
  //const FString JsonMaterialsPath = FPaths::ProjectContentDir() + "/JsonFiles/materials.json";
  LoadReflectivityMapFromJson();

  //Cargar la lista de actores desde un archivo json 
  LoadActorsList();
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
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);

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
  
  channel_lidar = new ChannelLidar();
  channel_lidar->init(&params);

  rx_lidar = new RxLidarPulsed();
  rx_lidar->init(&params);
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

float ATimeResolvedLidar::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

ATimeResolvedLidar::FDetection ATimeResolvedLidar::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
{
  auto start = std::chrono::high_resolution_clock::now();

  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  const float Distance = Detection.point.Length();

  //Atenuacion atmosferica en base a la distancia, por defecto de CARLA
  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);
  float CosAngle = 1.0;
  float ReflectivityValue = 1.0;
  
  //MEJORAS DEL MODELO
  //Efecto del angulo del incidencia

  if (Description.INTENSITY_CALC)
    {
      
      //Posicion del sensor
      FVector SensorLocation = SensorTransf.GetLocation(); 
      //Vector incidente, normalizado, entre sensor y punto de hit con el target
      FVector VectorIncidente = - (HitPoint - SensorLocation).GetSafeNormal();
      FVector VectorIncidente_t = VectorIncidente * Distance;
  
      //Vector normal a la superficie de hit, normalizado
      FVector VectorNormal = HitInfo.ImpactNormal;
      //Producto punto entre ambos vector, se obtiene el coseno del ang de incidencia
      CosAngle = FVector::DotProduct(VectorIncidente, VectorNormal);
      //CosAngle = sqrtf(CosAngle);
  
      //Efecto de la reflectividad del material
      AActor* ActorHit = HitInfo.GetActor();
      FString ActorHitName = ActorHit->GetName();

      const double* ReflectivityPointer;
      bool MaterialFound=false;
      bool ActorFound = false;

      //Determinar si el actor del hit, esta dentro de los actores a los cuales computar los materiales
      for (int32 i=0; i!=ActorsList.Num();i++){
	      if(ActorHitName.Contains(ActorsList[i])){
	        ActorFound=true;
	        break;
	      }
      }

      float MaterialSpecular = GetHitMaterialSpecular(HitInfo);
      if(params.DEBUG_GLOBAL)
	      UE_LOG(LogTemp, Log, TEXT("Specular: %s"), MaterialSpecular);
      //Segun si el nombre del actor, corresponde a un actor al cual computar su material
      if( ActorFound ){
	      //Se obtiene el nombre del material del hit
	      FString MaterialNameHit = GetHitMaterialName(HitInfo);
	      //FString MaterialSpecular = GetHitMaterialName(HitInfo);

	    if(params.DEBUG_GLOBAL)
	      UE_LOG(LogTemp, Log, TEXT("Material: %s"), *MaterialNameHit);

	    //Se recorre la lista de materiales con su respectiva reflectividad
	    for (auto& Elem : ReflectivityMap){
	      FString MaterialKey = Elem.Key;
	      //comprueba de si el nombre del material esta incluido en el material del hit
	      if(MaterialNameHit.Contains(MaterialKey)){
	        //cuando se encuentra, se obtiene el valor de reflectividad asociado a ese material
	        ReflectivityValue = (float)Elem.Value;
	        if(params.DEBUG_GLOBAL)
	          UE_LOG(LogTemp, Log, TEXT("Reflectividad: %f"), ReflectivityValue);
	        MaterialFound=true;
	        //WriteFile(MaterialNameHit);
	        break;
	        }
	      }
      }
      if(!MaterialFound){
	    //Se le asigna una reflectivdad por defeto a los materiales no criticos
	      ReflectivityPointer = ReflectivityMap.Find(TEXT("NoMaterial"));
	      ReflectivityValue = (float)*ReflectivityPointer;
      }

      //La intensidad del punto tiene en cuenta:
      //Atenuacion atmosferica -> la intensidad sera menor a mayor distancia
      //Cos Ang Incidencia -> la intensidad mientras mas perpendicular a la superficie sea el rayo incidente

      if (Description.TRANS_ON){
	      // LiDAR Transceptor
	      vector<float> output_tx;
	      output_tx = tx_lidar->run();
      
	      vector<float> output_channel;
	      output_channel = channel_lidar->run(output_tx,Distance,MaterialSpecular,CosAngle); // Ojo calcula la intensidad diferente
      
	      vector<float> output_rx;  
	      output_rx = rx_lidar->run(output_tx,output_channel);
	      Detection.time_signal = output_rx;
      
	      // Calculo de la distancia
	      auto it = max_element(output_rx.begin(),output_rx.end());
	      int max_idx = distance(output_rx.begin(),it);
	      double max_value = *it;
	      double distance = ((max_idx+1-output_tx.size())/(params.RX_FS*params.RX_NOS))*LIGHT_SPEED/2;      // Calculo de la distancia
	      FVector vector_proc = (VectorIncidente*distance);

	      // Only Debug
	      if(params.DEBUG_GLOBAL){
	        if (params.LOG_RX){
		        cout << "Punto: " << Detection.point.x << " " << Detection.point.y << " " << Detection.point.z << endl;
		        cout << "Punto: " << vector_proc.X << " " << vector_proc.Y << " " << vector_proc.Z << endl;
		        cout << output_tx.size() << " " << output_channel.size() << " " << endl;
		        UE_LOG(LogTemp, Log, TEXT("Distancia: %f"), Distance);
		        cout << "Distancia Receptor: " << distance << endl;
		        UE_LOG(LogTemp, Log, TEXT("Vector3: %s"), *(VectorIncidente*distance).ToString());
		        UE_LOG(LogTemp, Log, TEXT("Vector: %s"), *VectorIncidente.ToString());
		  
		        UE_LOG(LogTemp, Log, TEXT("Vector2: %s"), *VectorIncidente_t.ToString());
		  
		        cout << "******************* Detección ***************" << endl;
		        for (auto& i : Detection.time_signal)
		          cout <<  i << " ";
		        cout << endl;
		      }
	      }  
        Detection.point.x = vector_proc.X;
	      Detection.point.y = vector_proc.Y;
	      Detection.point.z = -vector_proc.Z;
	    }
    }

  //Reflectividad del material
  Detection.intensity = CosAngle * AbsAtm * ReflectivityValue;

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
    // Removed Extra Noise
    //if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon()) {
    //  if ( abs(Detection.point.x) < 0.0001 &&
    //	   abs(Detection.point.y) < 0.0001 &&
    //	   abs(Detection.point.z) < 0.0001 )
    //	Detection.point.z = 1.0;
    //  
    //  const auto ForwardVector = Detection.point.MakeUnitVector();
    //  const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
    //  Detection.point += Noise;
    //}

    const float Intensity = Detection.intensity;
    if(Intensity > Description.DropOffIntensityLimit)
      return true;
    else
      return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
  }

  void ATimeResolvedLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
      PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();

    LidarData.ResetMemory(PointsPerChannel);

    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
      for (auto& hit : RecordedHits[idxChannel]) {
        FDetection Detection = ComputeDetection(hit, SensorTransform);
        if (PostprocessDetection(Detection))
	        LidarData.WritePointSync(Detection);	    
        else
          PointsPerChannel[idxChannel]--;
      }
    }

    LidarData.WriteChannelCount(PointsPerChannel);
  }

  //Funcion implementada para leer desde un json, la reflectividad asociada a cada material
  //y cargarlo en el ReflectivityMap
  void ATimeResolvedLidar::LoadReflectivityMapFromJson(){

    //path del archivo json
    const FString FilePath = FPaths::ProjectDir() + "/LidarModelFiles/materials.json";
  
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
          ReflectivityMap.Add(name,reflec);

          GLog->Log("name:" + name);
          GLog->Log("reflectivity:" + FString::SanitizeFloat(reflec));
        }
      }
    }
  }

  void ATimeResolvedLidar::LoadActorsList(){

    //path del archivo json
    const FString FilePath = FPaths::ProjectDir() + "/LidarModelFiles/vehicles.json";
  
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

          ActorsList.Add(name);

          GLog->Log("name:" + name);

        }
      }
    }

  }

  bool ATimeResolvedLidar::WriteFile(FString Filename, FString String) {
    const FString FilePath = FPaths::ProjectContentDir() + TEXT("/LogFiles/") + Filename;

    FString new_String = FString::Printf( TEXT( "%s \n" ), *String);
    FFileHelper::SaveStringToFile(new_String, *FilePath,
    FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);

    return true;
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

  float ATimeResolvedLidar::GetHitMaterialSpecular(const FHitResult& HitInfo) const{

    UPrimitiveComponent* ComponentHit = HitInfo.GetComponent();
    
    if(ComponentHit){
      if (HitInfo.FaceIndex != -1) {
        int32 section = 0;
        UMaterialInterface* MaterialIntHit = ComponentHit->GetMaterialFromCollisionFaceIndex(HitInfo.FaceIndex, section);
	float ScalarValue;
	UMaterialInstanceDynamic* MaterialInstance = UMaterialInstanceDynamic::Create(MaterialIntHit, nullptr);
	bool bScalarFound = MaterialInstance->GetScalarParameterValue(FName("Specular"), ScalarValue);

        return ScalarValue;

      }
    }

    return -1;
    
  }