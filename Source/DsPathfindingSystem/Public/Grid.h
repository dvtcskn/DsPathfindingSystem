/*
* DsPathfindingSystem
* Plugin code
* Copyright 2017-2018 Gereksiz
* All Rights Reserved.
*/

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"	
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Components/BoxComponent.h"
#include "Grid.generated.h"

DECLARE_STATS_GROUP(TEXT("Grid"), STATGROUP_GRID, STATCAT_Advanced);

/*
* Node direction
*/
UENUM(BlueprintType)
enum class ENeighborDirection : uint8
{
	E_NOWHERE		UMETA(DisplayName = "NOWHERE"),
	E_NORTH			UMETA(DisplayName = "NORTH"),
	E_NORTH_EAST	UMETA(DisplayName = "NORTH_EAST"),
	E_EAST			UMETA(DisplayName = "EAST"),
	E_SOUTH_EAST	UMETA(DisplayName = "SOUTH_EAST"),
	E_SOUTH			UMETA(DisplayName = "SOUTH"),
	E_SOUTH_WEST	UMETA(DisplayName = "SOUTH_WEST"),
	E_WEST			UMETA(DisplayName = "WEST"),
	E_NORTH_WEST	UMETA(DisplayName = "NORTH_WEST"),
};

/*
* Search Result
*/
UENUM(BlueprintType)
enum class EAStarResultState : uint8
{
	SearchFail,
	SearchSuccess,
	GoalUnreachable,
	InfiniteLoop
};

/*
* GridType
*/
UENUM(BlueprintType)
enum class EGridType : uint8
{
	E_Square	UMETA(DisplayName = "Square"),
	E_Hex		UMETA(DisplayName = "Hex")
};

/*
* Search Functions returns
*/
USTRUCT(BlueprintType)
struct DSPATHFINDINGSYSTEM_API FSearchResult
{
	GENERATED_USTRUCT_BODY()

	/* Stores all found nodes locations */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		TArray<FVector> PathResults;
	/* Stores all found nodes indexes */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		TArray<int32> PathIndexes;
	/* Stores all found nodes index then Parents */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		TMap<int32, int32>	Parents;
	/* Stores all found nodes index then Node Costs */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		TMap<int32, float> PathCosts;
	/* Total path cost */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		float TotalNodeCost;
	/* Total path Length */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		int32 PathLength;
	/* Search state */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		EAStarResultState ResultState = EAStarResultState::SearchFail;

	FSearchResult()
		: TotalNodeCost(NULL)
		, PathLength(NULL)
		, ResultState(EAStarResultState::SearchFail)
	{};
};
/*
* struct FAStarResult : public FSearchResult
* To differentiate outputs.
*/
USTRUCT(BlueprintType)
struct DSPATHFINDINGSYSTEM_API FAStarResult : public FSearchResult { GENERATED_USTRUCT_BODY() };
/*
* struct FPSARResult : public FSearchResult
* To differentiate outputs.
*/
USTRUCT(BlueprintType)
struct DSPATHFINDINGSYSTEM_API FPSARResult  : public FSearchResult { GENERATED_USTRUCT_BODY() };

/*
* Node neighbors
*/
USTRUCT(BlueprintType)
struct DSPATHFINDINGSYSTEM_API FNeighbors
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		int32 EAST;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		int32 WEST;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		int32 SOUTH;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		int32 NORTH;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		int32 SOUTHEAST;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		int32 SOUTHWEST;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		int32 NORTHWEST;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		int32 NORTHEAST;

	bool Contains(int32 value) {
		if (value == EAST)
			return true;
		if (value == WEST)
			return true;
		if (value == SOUTH)
			return true;
		if (value == NORTH)
			return true;
		if (value == SOUTHEAST)
			return true;
		if (value == SOUTHWEST)
			return true;
		if (value == NORTHWEST)
			return true;
		if (value == NORTHEAST)
			return true;

		return false;
	};

	TMap<int32, float>  GetAllNodes() {
		TMap<int32, float>  temp;
		if (EAST != -1)
			temp.Add(EAST, 1.0f);
		if (WEST != -1)
			temp.Add(WEST, 1.0f);
		if (SOUTH != -1)
			temp.Add(SOUTH, 1.0f);
		if (NORTH != -1)
			temp.Add(NORTH, 1.0f);
		if (SOUTHEAST != -1)
			temp.Add(SOUTHEAST, 1.0f);
		if (NORTHWEST != -1)
			temp.Add(NORTHWEST, 1.0f);
		if (SOUTHWEST != -1)
			temp.Add(SOUTHWEST, 1.0f);
		if (NORTHEAST != -1)
			temp.Add(NORTHEAST, 1.0f);

		return temp;
	}

	FNeighbors()
		: EAST(-1)
		, WEST(-1)
		, SOUTH(-1)
		, NORTH(-1)
		, SOUTHEAST(-1)
		, SOUTHWEST(-1)
		, NORTHWEST(-1)
		, NORTHEAST(-1)
	{}
};

/*
* Search Functions Property
*/
USTRUCT(BlueprintType)
struct DSPATHFINDINGSYSTEM_API FAStarPreferences
{
	GENERATED_USTRUCT_BODY()

	/*
	* Node(Terrain) Cost Default 1.0f
	* Ignore all obstacles
	*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		uint32 bFly : 1;
	/*AStar Function Only*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		uint32 bStopAtNeighborLocation : 1;
	/*SQUARE GRID ONLY*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		uint32 bDiagonal : 1;
	/* Node(Terrain) Cost Default 1.0f */
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		uint32 bOverrideNodeCostToOne : 1;
	/*
	* Prototype ONLY
	* C++ and Blueprint communication is laggy
	*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		uint32 bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY : 1;

	FAStarPreferences()
		: bDiagonal(true)
	{}
};

/*
* Node Behavior
*/
USTRUCT(Blueprintable)
struct DSPATHFINDINGSYSTEM_API FNodeResult
{
	GENERATED_USTRUCT_BODY()

	/*
	* Is node accessible or not
	*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		uint32 bAccess : 1;

	/*
	* Defines how much cost to get in to the node.
	* Terrain Cost
	*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
		float NodeCost = 1.0f;
};

UCLASS(Blueprintable)
class DSPATHFINDINGSYSTEM_API AGrid : public AActor
{
	GENERATED_BODY()

protected:

	/*
	* Get Node location from grid
	*/
	FORCEINLINE FVector FIGetCellLocation(int32 index)
	{
		FTransform Result;
		hISM->GetInstanceTransform(index, Result, true);
		return Result.GetLocation();
	}

	/*
	* Heuristic Function
	*/
	FORCEINLINE float FIOctileDistance(FVector firstVector, FVector secondVector, float D = 1.0f)
	{
		float dx = FMath::Abs(firstVector.X - secondVector.X);
		float dy = FMath::Abs(firstVector.Y - secondVector.Y);
		return D * (dx + dy) + (FMath::Sqrt(2) - 2 * D) * FMath::Min(dx, dy);
	}

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
public:	
	// Sets default values for this actor's properties
	AGrid();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	/*
	* Grid creation
	*/
	virtual void OnConstruction(const FTransform& Transform) override;

	/*
	* Node cost and access logic.
	* NeighborIndex is important. You need to return NeighborIndex(Node) values.
	* The direction tells the neighbor index to go according to the Current Index.
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Logic")
		virtual FNodeResult NodeBehavior(int32 CurrentIndex, int32 NeighborIndex, ENeighborDirection Direction = ENeighborDirection::E_NOWHERE);

	/*
	* Node cost and access logic
	* NeighborIndex is important. You need to return NeighborIndex values.
	* The direction tells the neighbor index to go according to the Current Index.
	* Prototype ONLY
	* C++ and Blueprint communication is laggy
	*/
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "DsPathfindingSystem|Blueprint")
		FNodeResult NodeBehaviorBlueprint(int32 CurrentIndex, int32 NeighborIndex, ENeighborDirection Direction = ENeighborDirection::E_NOWHERE);
	virtual FNodeResult NodeBehaviorBlueprint_Implementation(int32 CurrentIndex, int32 NeighborIndex, ENeighborDirection Direction = ENeighborDirection::E_NOWHERE);

	/* returns UHierarchicalInstancedStaticMeshComponent */
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
		UHierarchicalInstancedStaticMeshComponent* getHISM();

	/* GridX size */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = 0, ClampMax = 256), Category = "DsPathfindingSystem")
		int32 GridX;
	/* GridY size */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = 0, ClampMax = 256), Category = "DsPathfindingSystem")
		int32 GridY;

	/*
	* Defines Hex or Square
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem")
		EGridType gridType;

	/*
	* In editor you can't actually select UHierarchicalInstancedStaticMeshComponent
	* when you try to select UHierarchicalInstancedStaticMeshComponent it's start to load all nodes(128*128) ~16k
	* then editor start to freeze 
	* because of that we need to some Dummy for collaision
	*/
	UPROPERTY(VisibleDefaultsOnly, BlueprintReadWrite, Category = "DsPathfindingSystem|Grid")
		UBoxComponent* DummyComponentforCollision;

	/*
	* Main pathfinding function
	* For accurate pathfinding (e.g follow road) you need to set NodeCostScale to big number 
	* if terrain cost is 1.0 this can be 1000
	* if terrain cost is 100.0 this can be 10
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|AStar")
		FAStarResult AStarSearch(int32 startIndex, int32 endIndex, FAStarPreferences Preferences, float NodeCostScale = 1000.0f);

	/*
	* Finds all accessible nodes at range
	* For accurate pathfinding (e.g follow road) you need to set default terrain cost value
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|AStar")
		FPSARResult PathSearchAtRange(int32 startIndex, int32 atRange, FAStarPreferences Preferences, float DefaultNodeCost = 1.0f);
	/*
	* For PathSearchAtRange function reconstructing paths
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|AStar")
		FPSARResult retracePath(int32 startIndex, int32 endIndex, FPSARResult StructData);

	/*
	* Return neighbor node indexes with given index value
	*		With Diagonal algorithm
	*
	*			+---+---+---+
	*			| NW| N | NE|
	*			+---+---+---+
	*			| W |   | E |
	*			+---+---+---+
	*			| SW| S | SE|
	*			+---+---+---+
	*	EAST and WEST SQUARE Grid Only
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
		TMap<int32, float> GetNeighborIndexes(int32 index, FAStarPreferences Preferences);

	/*
	* Return neighbor node indexes with given index value without checking is accessible
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
		FNeighbors calculateNeighborIndexes(int32 index);

	/*
	* returns neighbor node direction according to CurrentIndex
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
		ENeighborDirection GetNodeDirection(int32 CurrentIndex, int32 NextIndex);

	/*
	* Get Node location from grid
	* UHierarchicalInstancedStaticMeshComponent::GetInstanceTransform()
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
		FVector pureGetCellLocation(int32 index);

	/*
	* Get Node locations from grid using sphere selection
	* UHierarchicalInstancedStaticMeshComponent::GetInstancesOverlappingSphere()
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
		TArray<int32> pureGetCellIndexWithSphere(FVector Center, float Radius);

	/*
	* Get Node locations from grid using box selection
	* box auto generated
	* UHierarchicalInstancedStaticMeshComponent::GetInstancesOverlappingBox()
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
		TArray<int32> pureGetCellIndexWithBox(FVector Center, float Radius);

	/*
	* Finds node index with LineTraceMultiByObjectType()
	* using UHierarchicalInstancedStaticMeshComponent::GetCollisionObjectType()
	* collision same with DummyComponentforCollision
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
		int32 getNodeIndex(FVector targetVector);

	/*
	* Returns given Actor Z value
	* If there is no Actor in the given place. Its returns -BIG_NUMBER.
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
		float getActorZ(AActor * Actor, FVector Location, ECollisionChannel TraceType);

protected:

	/* UHierarchicalInstancedStaticMeshComponent is attached to scene*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem")
		USceneComponent* scene;

	/*
	* Gird location 
	* not confuse with scene or Actor lcoation
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem")
		FVector gridLoc;
	/*
	* Grid cell(Nodes) size
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem")
		FVector cellScale;
	/*
	* Grid cell(Nodes) mesh
	* e.g for square grid use square mesh for hexagon use hexagon mesh :)
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem")
		UStaticMesh* GridMesh;

	/* Gap between nodes X axis*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem")
		float GridOffsetX;
	/* Gap between nodes Y axis*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem")
		float GridOffsetY;
	/* (GirdX * GridY) - 1 */
	UPROPERTY(BlueprintReadOnly, Category = "DsPathfindingSystem|Grid")
		int32 TotalGridSize;

	/*Grid main component*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Grid")
		UHierarchicalInstancedStaticMeshComponent* hISM;

	/* Hex grid or not */
	UPROPERTY(BlueprintReadOnly, Category = "DsPathfindingSystem|Grid")
		bool bHex;
	/* Grid visible or not */
	UPROPERTY(EditAnywhere, Category = "DsPathfindingSystem")
		bool bVisible;
};
