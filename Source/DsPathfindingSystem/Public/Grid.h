/*
* DsPathfindingSystem
* Plugin code
* Copyright (c) 2024 Davut Coşkun
* All Rights Reserved.
*/

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"
#include "Grid.generated.h"

DECLARE_STATS_GROUP(TEXT("Grid"), STATGROUP_GRID, STATCAT_Advanced);
//DSPATHFINDINGSYSTEM_API DECLARE_LOG_CATEGORY_EXTERN(LogGridError, Error, All);

/*
* Node direction
*/
UENUM(BlueprintType)
enum class ENeighborDirection : uint8
{
	None			UMETA(DisplayName = "NOWHERE"),
	NORTH			UMETA(DisplayName = "NORTH"),
	NORTH_EAST		UMETA(DisplayName = "NORTH_EAST"),
	EAST			UMETA(DisplayName = "EAST"),
	SOUTH_EAST		UMETA(DisplayName = "SOUTH_EAST"),
	SOUTH			UMETA(DisplayName = "SOUTH"),
	SOUTH_WEST		UMETA(DisplayName = "SOUTH_WEST"),
	WEST			UMETA(DisplayName = "WEST"),
	NORTH_WEST		UMETA(DisplayName = "NORTH_WEST"),
};

/*
* Search Result
*/
UENUM(BlueprintType)
enum class EAStarResultState : uint8
{
	SearchFail		UMETA(DisplayName = "Search Fail"),
	SearchSuccess	UMETA(DisplayName = "Search Success"),
	GoalUnreachable	UMETA(DisplayName = "Goal Unreachable"),
	InfiniteLoop	UMETA(DisplayName = "Infinite Loop")
};

UENUM(BlueprintType)
enum class ETileType : uint8
{
	Grass		UMETA(DisplayName = "Grass"),
	Water		UMETA(DisplayName = "Water"),
	// lava, dirt, desert, road(?) etc
};

UENUM(BlueprintType)
enum class ESearchType : uint8
{
	Ground		UMETA(DisplayName = "Ground"),
	Water		UMETA(DisplayName = "Water"),
	/*
	* Node(Terrain) Cost Default 1.0f
	* Ignore all obstacles
	*/
	Fly			UMETA(DisplayName = "Fly"),
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
	GENERATED_BODY()

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
	/* Stores all found obstacles indexes */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	TArray<int32> ObstacleIndexes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bIsDiagonal : 1;
	/* Search state */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	EAStarResultState ResultState = EAStarResultState::SearchFail;

	FSearchResult()
		: TotalNodeCost(NULL)
		, PathLength(NULL)
		, bIsDiagonal(false)
		, ResultState(EAStarResultState::SearchFail)
	{};
	FSearchResult(EAStarResultState ResultState)
		: TotalNodeCost(NULL)
		, PathLength(NULL)
		, bIsDiagonal(false)
		, ResultState(ResultState)
	{};
};

/*
* Search Functions returns
*/
USTRUCT(BlueprintType)
struct DSPATHFINDINGSYSTEM_API FTileNeighborResult
{
	GENERATED_BODY()

	/* Stores all found nodes locations */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	TMap<int32, float> Neighbors;
	/* Stores all found nodes indexes */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	TArray<int32> ObstacleIndexes;

	FTileNeighborResult()
	{};
};

/*
* Node neighbors
*/
USTRUCT(BlueprintType)
struct DSPATHFINDINGSYSTEM_API FNeighbors
{
	GENERATED_BODY()

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

	inline bool Contains(int32 value) const
	{
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

	/*
	* Diagonal Square Grid only
	*/
	inline TMap<int32, float>  GetAllNodes(EGridType GridType = EGridType::E_Square, bool bSquareGridDiagonal = false) const
	{
		TMap<int32, float>  temp;

		if (GridType == EGridType::E_Square && !bSquareGridDiagonal)
		{
			if (EAST != -1)
				temp.Add(EAST, 1.0);
			if (WEST != -1)
				temp.Add(WEST, 1.0);
			if (SOUTH != -1)
				temp.Add(SOUTH, 1.0);
			if (NORTH != -1)
				temp.Add(NORTH, 1.0);
			return temp;
		}

		if (EAST != -1 && GridType == EGridType::E_Square)
			temp.Add(EAST, 1.0f);
		if (WEST != -1 && GridType == EGridType::E_Square)
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

	inline void Filter(int32 GridSize)
	{
		if (EAST > GridSize || EAST < 0)
			EAST = -1;
		if (WEST > GridSize || WEST < 0)
			WEST = -1;
		if (SOUTH > GridSize || SOUTH < 0)
			SOUTH = -1;
		if (NORTH > GridSize || NORTH < 0)
			NORTH = -1;
		if (SOUTHEAST > GridSize || SOUTHEAST < 0)
			SOUTHEAST = -1;
		if (NORTHWEST > GridSize || NORTHWEST < 0)
			NORTHWEST = -1;
		if (SOUTHWEST > GridSize || SOUTHWEST < 0)
			SOUTHWEST = -1;
		if (NORTHEAST > GridSize || NORTHEAST < 0)
			NORTHEAST = -1;
	}
};

/*
* Search Functions Property
*/
USTRUCT(BlueprintType)
struct DSPATHFINDINGSYSTEM_API FAStarPreferences
{
	GENERATED_BODY()

	/*SQUARE GRID ONLY*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bDiagonal : 1;
	/* Node(Terrain) Cost Default 1.0f */
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bOverrideNodeCostToOne : 1;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bIgnorePlayerCharacters : 1;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	int32 PlayerIDToIgnore;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	TArray<int32> PlayerIDsToIgnore;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bRecordObstacleIndexes : 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	TArray<int32> TileIndexesToFilter;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 IgnoreTileType : 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 IgnoreTileObstackle : 1;
	/*
	* Prototype ONLY
	* C++ and Blueprint communication is laggy
	*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY : 1;

	FAStarPreferences()
		: bDiagonal(true)
		, bOverrideNodeCostToOne(false)
		, bIgnorePlayerCharacters(false)
		, PlayerIDToIgnore(0)
		, bRecordObstacleIndexes(false)
		, IgnoreTileType(false)
		, IgnoreTileObstackle(false)
		, bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY(false)
	{}
};

/*
* Node Behavior
*/
USTRUCT(Blueprintable)
struct DSPATHFINDINGSYSTEM_API FNodeProperty
{
	GENERATED_BODY()

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

	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	ETileType TileType;
	
	FNodeProperty()
		: bAccess(true)
		, NodeCost(1.0f)
		, TileType(ETileType::Grass)
	{}
	FNodeProperty(uint32 Access, float Cost, ETileType TileType)
		: bAccess(Access)
		, NodeCost(Cost)
		, TileType(TileType)
	{}
};

/*
* Node
*/
USTRUCT(Blueprintable)
struct DSPATHFINDINGSYSTEM_API FGridNode
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	bool bIsValid;

	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	FVector Location;

	//UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	//FBox Bound;

	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	FNodeProperty NodeProperty;

	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	AActor* Actor;

	FGridNode()
		: bIsValid(true)
		, Location(FVector::ZeroVector)
		//, Bound(FBox())
		, NodeProperty(FNodeProperty())
		, Actor(nullptr)
	{}
	FGridNode(FVector Loc, FNodeProperty Property = FNodeProperty())
		: bIsValid(true)
		, Location(Loc)
		//, Bound(Bound)
		, NodeProperty(Property)
		, Actor(nullptr)
	{}
	//FGridNode(FVector Loc, /*FBox Bound,*/ FNodeProperty Property)
	//	: Location(Loc)
		//, Bound(Bound)
	//	, NodeProperty(Property)
	//{}
};

UCLASS(Blueprintable)
class DSPATHFINDINGSYSTEM_API AGrid : public AActor
{
	GENERATED_BODY()
protected:

	/*
	* Heuristic Function
	*/
	FORCEINLINE float FIOctileDistance(FVector firstVector, FVector secondVector, float D = 1.0f) const
	{
		const float dx = FMath::Abs(firstVector.X - secondVector.X);
		const float dy = FMath::Abs(firstVector.Y - secondVector.Y);
		return D * (dx + dy) + (FMath::Sqrt(2.0f) - 2 * D) * FMath::Min(dx, dy);
	}

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Sets default values for this actor's properties
	AGrid();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	void GenerateGrid(EGridType Type, int32 InGridX, int32 InGridY, TMap<int32, FNodeProperty> NodeProperties, bool bUseCustomTileBounds, FBox InCustomTileBounds, FVector2D InTileOffset = FVector2D::ZeroVector, FVector2D InTileScale = FVector2D(1.0f, 1.0f), bool bUseCustomGridLocation = false, FVector CustomGridLocation = FVector::ZeroVector);

	/*
	* Main pathfinding function
	* For accurate pathfinding (e.g follow road) you need to set NodeCostScale to big number
	* if terrain cost is 1.0 this can be 1000
	* if terrain cost is 100.0 this can be 10
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|AStar")
	FSearchResult AStarSearch(int32 startIndex, int32 endIndex, FAStarPreferences Preferences, bool bStopAtNeighborLocation = false, ESearchType SearchType = ESearchType::Ground, float NodeCostScale = 1000.0f);

	/*
	* Finds all accessible nodes at range
	* For accurate pathfinding (e.g follow road) you need to set default terrain cost value
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|AStar")
	FSearchResult PathSearchAtRange(int32 startIndex, int32 atRange, FAStarPreferences Preferences, ESearchType SearchType = ESearchType::Ground, float DefaultNodeCost = 1.0f);
	/*
	* For PathSearchAtRange function reconstructing paths
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|AStar", meta = (AutoCreateRefTerm = "NeighborIndexesToFilter"))
	FSearchResult RetracePath(int32 startIndex, int32 endIndex, bool bStopAtNeighborLocation, TArray<int32> NeighborIndexesToFilter, FSearchResult StructData);
	
	/*
	* Get Node locations from grid using sphere selection
	* UHierarchicalInstancedStaticMeshComponent::GetInstancesOverlappingSphere()
	* BP pure
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	TArray<int32> GetTileIndexWithSphere(FVector Center, float Radius) const;

	/*
	* Get Node locations from grid using box selection
	* box auto generated
	* UHierarchicalInstancedStaticMeshComponent::GetInstancesOverlappingBox()
	* BP pure
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	TArray<int32> GetTileIndicesWithBox(FVector Center, FVector Extent) const;

	/*
	* Get Node location from grid
	* UHierarchicalInstancedStaticMeshComponent::GetInstanceTransform()
	* BP only (pure)
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	FVector GetTileLocation(int32 index) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	FBox GetTileBox(int32 index) const;

	/* 
	* BP only (pure)
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	FGridNode GetTileByIndex(int32 index) const;

	/*
	* Return neighbor node indexes with given index value without checking is accessible
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	FNeighbors CalculateNeighborIndexes(int32 index) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	TArray<int32> CalculateNeighborIndexesAsArray(int32 index, bool bSquareGridDiagonal = false) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	int32 GetNeighborIndex(int32 index, ENeighborDirection Direction, ESearchType SearchType = ESearchType::Ground) const;

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
	FTileNeighborResult GetNeighborIndexes(int32 index, FAStarPreferences Preferences, ESearchType SearchType = ESearchType::Ground) const;

	/*
	* Node cost and access logic.
	* NeighborIndex is important. You need to return NeighborIndex(Node) values.
	* The direction tells the neighbor index to go according to the Current Index.
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Logic")
	virtual FNodeProperty NodeBehavior(int32 CurrentIndex, int32 NeighborIndex, FAStarPreferences Preferences, ESearchType SearchType = ESearchType::Ground, ENeighborDirection Direction = ENeighborDirection::None) const;

	/*
	* Node cost and access logic
	* NeighborIndex is important. You need to return NeighborIndex values.
	* The direction tells the neighbor index to go according to the Current Index.
	* Prototype ONLY
	* C++ and Blueprint communication is laggy
	*/
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "DsPathfindingSystem|Blueprint")
	FNodeProperty NodeBehaviorBlueprint(int32 CurrentIndex, int32 NeighborIndex, FAStarPreferences Preferences, ESearchType SearchType = ESearchType::Ground, ENeighborDirection Direction = ENeighborDirection::None) const;
	virtual FNodeProperty NodeBehaviorBlueprint_Implementation(int32 CurrentIndex, int32 NeighborIndex, FAStarPreferences Preferences, ESearchType SearchType = ESearchType::Ground, ENeighborDirection Direction = ENeighborDirection::None) const;

	/*
	* returns neighbor node direction according to CurrentIndex
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	ENeighborDirection GetNodeDirection(int32 CurrentIndex, int32 NextIndex) const;

	/*
	* Finds node index with LineTraceMultiByObjectType()
	* using UHierarchicalInstancedStaticMeshComponent::GetCollisionObjectType()
	* collision same with DummyComponentforCollision
	*/
	//UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	//	int32 getNodeIndex(FVector targetVector);

	/*
	* Returns given Actor Z value
	* If there is no Actor in the given place. Its returns -BIG_NUMBER.
	*/
	//UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	//float getActorZ(AActor* Actor, FVector Location, ECollisionChannel TraceType);

	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool SetTileProperty(int32 index, FNodeProperty Property);
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool SetTilePropertyMap(TMap<int32, FNodeProperty> NodeProperties);

	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	void RegisterActorToTile(int32 index, AActor* Actor);
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	void UnregisterActorFromTile(int32 index);

	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	void ClearInstances();

private:
	TArray<int32> GetInstancesOverlappingBox(const FBox& Box) const;
	TArray<int32> GetInstancesOverlappingSphere(const FVector& Center, const float Radius) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	int32 GetTileIndex(FVector Point) const;

	void AddInstance(FVector Instance, FNodeProperty NodeProperty);
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	int32 GetGridSize() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	FBox GetTileBound() const;

	FORCEINLINE FGridNode GetTile(int32 index) const
	{
		if (Instances.Num() == 0)
		{
			//UE_VLOG(GetOwner(), LogGridError, Log, TEXT("You are trying to access a tile on an empty grid."));
			FGridNode InvalidNode;
			InvalidNode.bIsValid = false;
			InvalidNode.Location = FVector::ZeroVector;
			InvalidNode.NodeProperty.bAccess = false;
			InvalidNode.NodeProperty.NodeCost = 9999999;
			return InvalidNode;
		}

		if (index >= Instances.Num())
			return Instances[Instances.Num() - 1];
		else if (index < 0)
			return Instances[0];
		return Instances[index];
	}

protected:

	/* UHierarchicalInstancedStaticMeshComponent is attached to scene*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem")
	USceneComponent* scene;

	/*
	* Defines Hex or Square
	*/
	UPROPERTY(BlueprintReadOnly, Category = "DsPathfindingSystem|Grid")
	EGridType gridType;

	/* GridX size */
	UPROPERTY(BlueprintReadOnly, Category = "DsPathfindingSystem|Grid")
	int32 GridX;
	/* GridY size */
	UPROPERTY(BlueprintReadOnly, Category = "DsPathfindingSystem|Grid")
	int32 GridY;

	/* Gap between nodes XY axis*/
	UPROPERTY(BlueprintReadOnly, Category = "DsPathfindingSystem|Grid")
	FVector2D TileOffset;

	/* */
	UPROPERTY(BlueprintReadOnly, Category = "DsPathfindingSystem|Grid")
	FBox TileBound;

	/*
	* Grid Tile(Nodes) size
	*/
	UPROPERTY(BlueprintReadOnly, Category = "DsPathfindingSystem")
	FVector2D TileScale;

	UPROPERTY(BlueprintReadOnly, Category = "DsPathfindingSystem|Grid")
	TMap<int32, FGridNode> Instances;
};
