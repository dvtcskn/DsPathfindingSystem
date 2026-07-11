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
#include "DsGrid.generated.h"

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

UENUM(BlueprintType)
enum class ESearchFunction : uint8
{
	ASTAR				UMETA(DisplayName = "ASTAR"),
	PathSearchAtRange	UMETA(DisplayName = "PathSearchAtRange"),
};

/*
* Search Result
*/
UENUM(BlueprintType)
enum class ESearchResult : uint8
{
	SearchFail		UMETA(DisplayName = "Search Fail"),
	SearchSuccess	UMETA(DisplayName = "Search Success"),
	AlreadyAtGoal	UMETA(DisplayName = "Already At Goal"),
	GoalUnreachable	UMETA(DisplayName = "Goal Unreachable"),
	InfiniteLoop	UMETA(DisplayName = "Infinite Loop")
};

UENUM(BlueprintType)
enum class EGridHeuristicFunction : uint8
{
	Octile			UMETA(DisplayName = "Octile"),
	Manhattan		UMETA(DisplayName = "Manhattan"),
	Euclidean		UMETA(DisplayName = "Euclidean"),
};

UENUM(BlueprintType)
enum class ETileType : uint8
{
	Undefined	UMETA(DisplayName = "Undefined"),
	Grass		UMETA(DisplayName = "Grass"),
	Water		UMETA(DisplayName = "Water"),
	// lava, dirt, desert etc
};

UENUM(BlueprintType)
enum class EGridType : uint8
{
	Square	UMETA(DisplayName = "Square"),
	Hex		UMETA(DisplayName = "Hex")
};

UENUM(BlueprintType)
enum class EGridTileOrder : uint8
{
	RowMajor		UMETA(DisplayName = "RowMajor"),
	ColumnMajor		UMETA(DisplayName = "ColumnMajor")
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
	int32 EndPoint;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bStopAtNeighborLocation : 1;
	/* Search state */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	ESearchResult ResultState = ESearchResult::SearchFail;

	FSearchResult()
		: TotalNodeCost(NULL)
		, PathLength(NULL)
		, EndPoint(-1)
		, bStopAtNeighborLocation(false)
		, ResultState(ESearchResult::SearchFail)
	{};
	FSearchResult(ESearchResult NewResultState)
		: TotalNodeCost(NULL)
		, PathLength(NULL)
		, EndPoint(-1)
		, bStopAtNeighborLocation(false)
		, ResultState(NewResultState)
	{};

	int32 GetLastIndex() const
	{
		if (PathIndexes.Num() > 0)
			return PathIndexes.Last();
		return -1;
	}
};

USTRUCT(BlueprintType)
struct FTileNeighborCost
{
	GENERATED_BODY()
public:
	float NodeCost;
	float NodeCostScale;

	inline float GetCost() { return NodeCost * NodeCostScale; }

	FTileNeighborCost()
		: NodeCost(1.0f)
		, NodeCostScale(1.0f)
	{}
	FTileNeighborCost(float NewNodeCost, float NewNodeCostScale = 1.0f)
		: NodeCost(NewNodeCost)
		, NodeCostScale(NewNodeCostScale)
	{}
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
	TMap<int32, FTileNeighborCost> Neighbors;
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

	inline bool Contains(int32 Index) const
	{
		if (Index == EAST)
			return true;
		if (Index == WEST)
			return true;
		if (Index == SOUTH)
			return true;
		if (Index == NORTH)
			return true;
		if (Index == SOUTHEAST)
			return true;
		if (Index == SOUTHWEST)
			return true;
		if (Index == NORTHWEST)
			return true;
		if (Index == NORTHEAST)
			return true;

		return false;
	};

	inline TArray<int32> GetAllNodesAsArray(EGridType GridType = EGridType::Square, bool bSquareGridDiagonal = false) const
	{
		TArray<int32> Indexes;

		if (GridType == EGridType::Square && !bSquareGridDiagonal)
		{
			if (EAST != -1)
				Indexes.Add(EAST);
			if (WEST != -1)
				Indexes.Add(WEST);
			if (SOUTH != -1)
				Indexes.Add(SOUTH);
			if (NORTH != -1)
				Indexes.Add(NORTH);
			return Indexes;
		}

		if (EAST != -1 && GridType == EGridType::Square)
			Indexes.Add(EAST);
		if (WEST != -1 && GridType == EGridType::Square)
			Indexes.Add(WEST);
		if (SOUTH != -1)
			Indexes.Add(SOUTH);
		if (NORTH != -1)
			Indexes.Add(NORTH);
		if (SOUTHEAST != -1)
			Indexes.Add(SOUTHEAST);
		if (NORTHWEST != -1)
			Indexes.Add(NORTHWEST);
		if (SOUTHWEST != -1)
			Indexes.Add(SOUTHWEST);
		if (NORTHEAST != -1)
			Indexes.Add(NORTHEAST);

		return Indexes;
	}

	inline void Filter(int32 GridSize)
	{
		if (EAST >= GridSize || EAST < 0)
			EAST = -1;
		if (WEST >= GridSize || WEST < 0)
			WEST = -1;
		if (SOUTH >= GridSize || SOUTH < 0)
			SOUTH = -1;
		if (NORTH >= GridSize || NORTH < 0)
			NORTH = -1;
		if (SOUTHEAST >= GridSize || SOUTHEAST < 0)
			SOUTHEAST = -1;
		if (NORTHWEST >= GridSize || NORTHWEST < 0)
			NORTHWEST = -1;
		if (SOUTHWEST >= GridSize || SOUTHWEST < 0)
			SOUTHWEST = -1;
		if (NORTHEAST >= GridSize || NORTHEAST < 0)
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

	/*
	* The Actor you are controlling.
	*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	TObjectPtr<AActor> Actor;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bOverrideNodeCostToOne : 1;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bBlockBorder : 1;

	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	TArray<int32> PlayerIDsToIgnore;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bIncreaseTileCostOfPlayerCharacters : 1;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	float TileCostScale;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	TArray<ETileType> TileTypesToIgnore;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	TArray<int32> TileIndexesToFilter;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bRecordObstacleIndexes : 1;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 IgnoreTileObstackle : 1;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	int32 TotalNodeCostLimit;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bFailIfTotalNodeCostExceeded : 1;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	uint32 bIgnoreEnemyUnitsIfCombatRatingExceeded : 1;
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	int32 TargetCombatRating;

	FAStarPreferences(AActor* NewAActor = nullptr)
		: Actor(NewAActor)
		, bOverrideNodeCostToOne(false)
		, bBlockBorder(true)
		//, ActorAffiliationToIgnore(EDSGridActorAffiliation::NONE)
		, bIncreaseTileCostOfPlayerCharacters(false)
		, TileCostScale(1.0f)
		, bRecordObstacleIndexes(false)
		, IgnoreTileObstackle(false)
		, TotalNodeCostLimit(-1)
		, bFailIfTotalNodeCostExceeded(false)
		, bIgnoreEnemyUnitsIfCombatRatingExceeded(false)
		, TargetCombatRating(5.0f)
	{}
};

/*
* Node Behavior
*/
USTRUCT(Blueprintable)
struct DSPATHFINDINGSYSTEM_API FNodeAttribute
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

	/*
	* Only affects search functionality. Not the actual tile cost.
	*/
	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	float NodeCostScale = 1.0f;
	
	FNodeAttribute()
		: bAccess(true)
		, NodeCost(1.0f)
		, TileType(ETileType::Grass)
		, NodeCostScale(1.0f)
	{}
	FNodeAttribute(uint32 NewAccess, float NewCost, ETileType NewTileType, float NewNodeCostScale = 1.0f)
		: bAccess(NewAccess)
		, NodeCost(NewCost)
		, TileType(NewTileType)
		, NodeCostScale(NewNodeCostScale)
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

	UPROPERTY(BlueprintReadWrite, Category = "DsPathfindingSystem|Structs")
	FNodeAttribute NodeAttribute;

	FGridNode()
		: bIsValid(true)
		, Location(FVector::ZeroVector)
		, NodeAttribute(FNodeAttribute())
	{}
	FGridNode(FVector NewLocation, FNodeAttribute NewAttribute = FNodeAttribute())
		: bIsValid(true)
		, Location(NewLocation)
		, NodeAttribute(NewAttribute)
	{}
};

UCLASS(Blueprintable)
class DSPATHFINDINGSYSTEM_API ADsGrid : public AActor
{
	GENERATED_BODY()

public:
	ADsGrid();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaTime) override;

public:
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	inline bool GenerateGrid(EGridType InGridType, int32 InGridX, int32 InGridY, bool bIsSquareGridDiagonalAllowed = false, bool bUseCustomTileBounds = false, FBox CustomTileBounds = FBox(), EGridTileOrder InTileOrder = EGridTileOrder::ColumnMajor)
	{
		return GenerateGridEx(InGridType, InGridX, InGridY, bIsSquareGridDiagonalAllowed, bUseCustomTileBounds, CustomTileBounds, InTileOrder, TMap<int32, FNodeAttribute>());
	}

	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool GenerateGridEx(EGridType InGridType, int32 InGridX, int32 InGridY, bool bIsSquareGridDiagonalAllowed, bool bUseCustomTileBounds, FBox CustomTileBounds, EGridTileOrder InTileOrder, TMap<int32, FNodeAttribute> NodeProperties, FVector2D InTileScale = FVector2D(1.0f, 1.0f), FVector2D InTileOffset = FVector2D::ZeroVector, bool bUseCustomGridLocation = false, FVector CustomGridLocation = FVector::ZeroVector);

	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	virtual bool Resize(int32 NewSizeX, int32 NewSizeY);

protected:
	virtual void OnGridGenerated() {}
	virtual void OnResize(int32 NewSizeX, int32 NewSizeY) {}

public:
	/*
	* Heuristic Functions
	*/
	FORCEINLINE float EuclideanDistance(FVector FirstVector, FVector SecondVector) const
	{
		return FMath::Sqrt(
			((FirstVector.X - SecondVector.X) * (FirstVector.X - SecondVector.X)) +
			((FirstVector.Y - SecondVector.Y) * (FirstVector.Y - SecondVector.Y))
		);
	}

	FORCEINLINE int32 ManhattanDistance(FVector FirstVector, FVector SecondVector) const
	{
		return FMath::Abs(FirstVector.X - SecondVector.X) + FMath::Abs(FirstVector.Y - SecondVector.Y);
	}

	FORCEINLINE float OctileDistance(FVector FirstVector, FVector SecondVector, float D = 1.0f) const
	{
		const float dx = FMath::Abs(FirstVector.X - SecondVector.X);
		const float dy = FMath::Abs(FirstVector.Y - SecondVector.Y);
		return D * (dx + dy) + (FMath::Sqrt(2.0f) - 2 * D) * FMath::Min(dx, dy);
	}

	FORCEINLINE float GetHeuristic(EGridHeuristicFunction HeuristicFunction, FVector FirstVector, FVector SecondVector, float D = 1.0f) const
	{
		switch (HeuristicFunction)
		{
		case EGridHeuristicFunction::Octile:
			return OctileDistance(FirstVector, SecondVector, D);
		case EGridHeuristicFunction::Manhattan:
			return (float)ManhattanDistance(FirstVector, SecondVector);
		case EGridHeuristicFunction::Euclidean:
			return EuclideanDistance(FirstVector, SecondVector);
		}
		return OctileDistance(FirstVector, SecondVector, D);
	}

public:
	/*
	* Main pathfinding function
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|AStar")
	FSearchResult AStarSearch(int32 StartIndex, int32 EndIndex, FAStarPreferences Preferences, bool bStopAtNeighborLocation = false, EGridHeuristicFunction HeuristicFunction = EGridHeuristicFunction::Octile) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	inline FSearchResult AStarSearch_Pure(int32 StartIndex, int32 EndIndex, FAStarPreferences Preferences, bool bStopAtNeighborLocation = false, EGridHeuristicFunction HeuristicFunction = EGridHeuristicFunction::Octile) const
	{
		return AStarSearch(StartIndex, EndIndex, Preferences, bStopAtNeighborLocation, HeuristicFunction);
	}

	/*
	* Finds all accessible nodes at range
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|AStar")
	FSearchResult PathSearchAtRange(int32 StartIndex, int32 AtRange, FAStarPreferences Preferences) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	inline FSearchResult PathSearchAtRange_Pure(int32 StartIndex, int32 AtRange, FAStarPreferences Preferences) const
	{
		return PathSearchAtRange(StartIndex, AtRange, Preferences);
	}

	/*
	* For PathSearchAtRange function reconstructing paths
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|AStar")
	FSearchResult RetracePath(int32 StartIndex, int32 EndIndex, FSearchResult StructData, bool bStopAtNeighborLocation = false) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	inline FSearchResult RetracePath_Pure(int32 StartIndex, int32 EndIndex, FSearchResult StructData, bool bStopAtNeighborLocation) const
	{
		return RetracePath(StartIndex, EndIndex, StructData, bStopAtNeighborLocation);
	}

	/*
	* Get Node locations from grid using sphere selection
	* BP pure
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	TArray<int32> GetTileIndexWithSphere(FVector Center, float Radius) const;

	/*
	* Get Node locations from grid using box selection
	* BP pure
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	TArray<int32> GetTileIndicesWithBox(FVector Center, FVector Extent) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	bool IsValidIndex(int32 Index) const;
	/*
	* Get Node location from grid
	* BP only (pure)
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	FVector GetTileLocation(int32 Index) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	FBox GetTileBox(int32 Index) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	int32 GetTileIndex(FVector Point) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	int32 GetGridSize() const;
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	int32 GetGridX() const { return GridX; }
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	int32 GetGridY() const { return GridY; }

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	FBox GetTileBound() const;

	UFUNCTION(BlueprintPure, Category = "DsPathfindingSystem")
	EGridTileOrder GetTileOrder() const { return TileOrder; }

	UFUNCTION(BlueprintPure, Category = "DsPathfindingSystem")
	int32 GetIndexRow(int32 Index) const;
	UFUNCTION(BlueprintPure, Category = "DsPathfindingSystem")
	int32 GetIndexColumn(int32 Index) const;

	UFUNCTION(BlueprintPure, Category = "DsPathfindingSystem")
	EGridType GetGridType() const { return GridType; }
	UFUNCTION(BlueprintPure, Category = "DsPathfindingSystem|Grid")
	ETileType GetTileType(int32 Index) const;

	/*
	* Return neighbor node indexes with given Index value without checking is accessible
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	FNeighbors GetNeighborTiles(int32 Index, bool bBlockBorder = true) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	TArray<int32> GetNeighborTilesAsArray(int32 Index, bool bBlockBorder = true) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	TArray<int32> GetNeighborTilesInRangeAsArray(int32 Index, int32 Range = 1, bool bBlockBorder = true) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	int32 GetNeighborTileIndex(int32 Index, ENeighborDirection Direction, ETileType InTileType = ETileType::Undefined) const;

	/*
	* Return neighbor node indexes with given Index value
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
	FTileNeighborResult GetNeighborIndexes(int32 CurrentIndex, int32 EndIndex, FAStarPreferences Preferences) const;

	/*
	* Node cost and access logic.
	* NeighborIndex is important. You need to return NeighborIndex(Node) values.
	* The direction tells the neighbor Index to go according to the Current Index.
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Logic")
	virtual FNodeAttribute NodeBehavior(int32 CurrentIndex, int32 NeighborIndex, int32 EndIndex, FAStarPreferences Preferences, ENeighborDirection Direction = ENeighborDirection::None) const;

	/*
	* returns neighbor node direction according to CurrentIndex
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|AStar")
	ENeighborDirection GetNodeDirection(int32 CurrentIndex, int32 NextIndex) const;

	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool SetTileType(int32 Index, ETileType NewTileType);
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool SetTileCost(int32 Index, float NewNodeCost);
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool SetTileAccess(int32 Index, bool NewTileAccess);
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool SetTileAttribute(int32 Index, ETileType NewTileType, float NewNodeCost = 1.0f, bool NewTileAccess = true);
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool SetTileAttributeByNode(int32 Index, FNodeAttribute NewProperty);
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool SetTilePropertyMap(TMap<int32, FNodeAttribute> NewNodeProperties);
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	bool SetTileZ(int32 Index, float Z);

protected:
	virtual void OnTileAttributeChanged(const FGridNode& Node) {}
	virtual void OnTilePropertyMapSet() {}

public:
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Grid")
	void ClearInstances();

	UFUNCTION(BlueprintPure)
	FORCEINLINE bool IsSquareGridDiagonalAllowed() const { return bSquareGridDiagonalAllowed; }

	/*
	* BP only (pure)
	*/
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "DsPathfindingSystem|Grid")
	FORCEINLINE FGridNode GetTile(int32 Index) const { return GetNode(Index); }

	FORCEINLINE const FGridNode& GetNode(int32 Index) const
	{
		if (Instances.Num() == 0)
		{
			static FGridNode InvalidNode(
				FVector::ZeroVector,
				FNodeAttribute(false, 9999999.0f, ETileType::Undefined)
			);
			InvalidNode.bIsValid = false;
			return InvalidNode;
		}

		if (Index >= Instances.Num())
			return Instances[Instances.Num() - 1];
		else if (Index < 0)
			return Instances[0];
		return Instances[Index];
	}

private:
	TArray<int32> GetInstancesOverlappingBox(const FBox& Box) const;
	TArray<int32> GetInstancesOverlappingSphere(const FVector& Center, const float Radius) const;

	void AddInstance(FVector Instance, FNodeAttribute NewNode);
	void CreateNewInstance(int32 Index, FNodeAttribute NewNode);

private:
	USceneComponent* Scene;
	EGridType GridType;
	EGridTileOrder TileOrder;
	int32 GridX;
	int32 GridY;
	FVector2D TileOffset;
	FBox TileBound;
	FVector2D TileScale;
	TMap<int32, FGridNode> Instances;
	bool bSquareGridDiagonalAllowed;
};
