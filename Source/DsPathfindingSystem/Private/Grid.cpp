/*
* DsPathfindingSystem
* Plugin code
* Copyright (c) 2024 Davut Coşkun
* All Rights Reserved.
*/

#include "Grid.h"

DECLARE_CYCLE_STAT(TEXT("Grid~ASTAR"), STAT_ASTARSEARCH, STATGROUP_GRID);
DECLARE_CYCLE_STAT(TEXT("Grid~PathSearchAtRange"), STAT_PathSearchAtRange, STATGROUP_GRID);
DECLARE_CYCLE_STAT(TEXT("Grid~ReTracePath"), STAT_ReTracePath, STATGROUP_GRID);
DECLARE_CYCLE_STAT(TEXT("Grid~neighbor"), STAT_GetNeighborIndexes, STATGROUP_GRID);
DECLARE_CYCLE_STAT(TEXT("Grid~GetNodeIndex"), STAT_GetNodeIndex, STATGROUP_GRID);
DECLARE_CYCLE_STAT(TEXT("Grid~GetCellLocation"), STAT_GetCellLocation, STATGROUP_GRID);
DECLARE_CYCLE_STAT(TEXT("Grid~GetNodeDirection"), STAT_GetNodeDirection, STATGROUP_GRID);
DECLARE_CYCLE_STAT(TEXT("Grid~NodeBehavior"), STAT_AccessNode, STATGROUP_GRID);
DECLARE_CYCLE_STAT(TEXT("Grid~NodeBehaviorBlueprint"), STAT_AccessNodeBlueprint, STATGROUP_GRID);
DECLARE_CYCLE_STAT(TEXT("Grid~CalcNeighbor"), STAT_CalcNeighbor, STATGROUP_GRID);

#define BoxBoundMin = FVector(-100.00000000000000, -100.00000000000000, -200.00001525878906);
#define BoxBoundMax = FVector(100.00000000000000, 100.00003051757812, 0.0000000000000000);

#define HexBoundMin = FVector( -86.602546691894531, -100.00000000000000, -46.808815002441406 );
#define HexBoundMax = FVector( 86.602546691894531, 100.00000000000000, 1.5258789062500000e-05 );

#define TotalGridSize (GridX * GridY) - 1

AGrid::AGrid()
	: Super()
	, gridType(EGridType::E_Square)
	, GridX(0)
	, GridY(0)
	, TileOffset(FVector2D(0.0, 0.0))
	, TileScale(FVector2D(1.0, 1.0))
{
	scene = CreateDefaultSubobject<USceneComponent>(TEXT("USceneComponent"));
	scene->SetMobility(EComponentMobility::Static);
	RootComponent = scene;

	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
}

// Called when the game starts or when spawned
void AGrid::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AGrid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AGrid::GenerateGrid(EGridType Type, int32 InGridX, int32 InGridY, TMap<int32, FNodeProperty> NodeProperties, bool bUseCustomTileBounds, FBox InCustomTileBounds, FVector2D InTileOffset, FVector2D InTileScale, bool bUseCustomGridLocation, FVector CustomGridLocation)
{
	ClearInstances();

	if (InGridX == 0 || InGridY == 0)
		return;

	gridType = Type;
	GridX = InGridX;
	GridY = InGridY;
	TileOffset = InTileOffset;
	TileScale = InTileScale;
	FVector gridLoc = bUseCustomGridLocation ? CustomGridLocation : GetActorLocation();

	TileBound = bUseCustomTileBounds ? InCustomTileBounds : gridType == EGridType::E_Square ? FBox(FVector(-100.00000000000000, -100.00000000000000, -200.00001525878906), FVector(100.00000000000000, 100.00003051757812, 0.0000000000000000)) : FBox(FVector(-86.602546691894531, -100.00000000000000, -46.808815002441406), FVector(86.602546691894531, 100.00000000000000, 1.5258789062500000e-05));

	float boundX = TileBound.Max.X - TileBound.Min.X + TileOffset.X;
	float boundY = TileBound.Max.Y - TileBound.Min.Y - TileOffset.Y;

	FVector NodeLocRef = FVector::ZeroVector;

	for (int32 ii = 0; ii <= TotalGridSize; ++ii)
	{
		switch (gridType)
		{
		case EGridType::E_Square:
		{
			NodeLocRef.X = ((ii / GridY) * boundX);
			NodeLocRef.Y = ((ii % GridY) * boundY);
			NodeLocRef.Z = 0.0f;
		}
		break;
		case EGridType::E_Hex:
		{
			NodeLocRef.X = ((((ii / GridX) % 2) * (boundX / 2)) + ((ii % GridX) * boundX));
			NodeLocRef.Y = ((ii / GridX) * (boundY * 0.75));
			NodeLocRef.Z = 0.0f;
		}
		break;
		}

		//NodeLocationArray.AddUnique(NodeLocRef * FVector(TileScale, 1.0f));
		if (NodeProperties.Contains(ii))
			AddInstance(gridLoc + ((NodeLocRef * FVector(TileScale, 1.0f))), NodeProperties[ii]);
		else
			AddInstance(gridLoc + ((NodeLocRef * FVector(TileScale, 1.0f))), FNodeProperty());
	}
}

TArray<int32> AGrid::GetTileIndexWithSphere(FVector Center, float Radius)
{
	return GetInstancesOverlappingSphere(Center, Radius);
}

TArray<int32> AGrid::GetTileIndicesWithBox(FVector Center, FVector Extent)
{
	FBox Box;
	Box.BuildAABB(Center, Extent);
	return GetInstancesOverlappingBox(Box);
}

FVector AGrid::GetTileLocation(int32 index)
{
	return GetTile(index).Location;
}

FBox AGrid::GetTileBox(int32 index)
{
	if (index < 0 || index > TotalGridSize)
		return FBox();
	return TileBound.MoveTo(GetTile(index).Location);
}

int32 AGrid::GetNeighborIndex(int32 index, ENeighborDirection Direction, FAStarPreferences Preferences, ESearchType SearchType)
{
	FTileNeighborResult Indices = GetNeighborIndexes(index, Preferences, SearchType);
	FNeighbors Neighbors = calculateNeighborIndexes(index);
	switch (Direction)
	{
	case ENeighborDirection::None:
		return -1;
	case ENeighborDirection::NORTH:
		if (Indices.Neighbors.Contains(Neighbors.NORTH))
			return Neighbors.NORTH;
		return -1;
	case ENeighborDirection::NORTH_EAST:
		if (Indices.Neighbors.Contains(Neighbors.NORTHEAST))
			return Neighbors.NORTHEAST;
		return -1;
	case ENeighborDirection::EAST:
		if (Indices.Neighbors.Contains(Neighbors.EAST))
			return Neighbors.EAST;
		return -1;
	case ENeighborDirection::SOUTH_EAST:
		if (Indices.Neighbors.Contains(Neighbors.SOUTHEAST))
			return Neighbors.SOUTHEAST;
		return -1;
	case ENeighborDirection::SOUTH:
		if (Indices.Neighbors.Contains(Neighbors.SOUTH))
			return Neighbors.SOUTH;
		return -1;
	case ENeighborDirection::SOUTH_WEST:
		if (Indices.Neighbors.Contains(Neighbors.SOUTHWEST))
			return Neighbors.SOUTHWEST;
		return -1;
	case ENeighborDirection::WEST:
		if (Indices.Neighbors.Contains(Neighbors.WEST))
			return Neighbors.WEST;
		return -1;
	case ENeighborDirection::NORTH_WEST:
		if (Indices.Neighbors.Contains(Neighbors.NORTHWEST))
			return Neighbors.NORTHWEST;
		return -1;
	}

	return -1;
}

FGridNode AGrid::GetTileByIndex(int32 index)
{
	return GetTile(index);
}

//int32 AGrid::getNodeIndex(FVector targetVector)
//{
//	SCOPE_CYCLE_COUNTER(STAT_GetNodeIndex);
//
//	/*UWorld* World	= GetWorld();
//	int32 out		= NULL;
//	float origin	= hISM->Bounds.Origin.Z;
//	float boxExtend = hISM->Bounds.BoxExtent.Z;
//
//	FVector sVector{ targetVector.X, targetVector.Y, ((boxExtend * 2) * -1.0f) + (origin + boxExtend) };
//	FVector eVector{ targetVector.X, targetVector.Y, origin + boxExtend };
//
//	TArray<FHitResult> traceHitResults;
//	FCollisionQueryParams traceParams;
//	traceParams.bTraceComplex = true;
//
//	World->LineTraceMultiByObjectType(traceHitResults, sVector, eVector, hISM->GetCollisionObjectType(), traceParams);
//
//	for (size_t i = 0; i < traceHitResults.Num(); i++)
//	{
//		if (traceHitResults[i].GetActor() != NULL) {
//			if (traceHitResults[i].GetActor() == this) {
//				if (traceHitResults[i].Item >= 0) {
//					out = traceHitResults[i].Item;
//					break;
//				}
//				else {
//					out = traceHitResults[i].Item + FMath::Abs(32767.0f) + 32769.0f;
//					break;
//				}
//			}
//		}
//	}
//	if (out <= ((GridX * GridY) - 1) && out >= 0)
//	{
//		return out;
//	}
//	else {
//		return -1;
//	}*/
//
//	return -1;
//}

//float AGrid::getActorZ(AActor* Actor, FVector Location, ECollisionChannel TraceType)
//{
//	float Z = -BIG_NUMBER;
//	if (UWorld* world = GetWorld()) {
//		TArray<FHitResult> traceHitResults;
//		if (Actor) {
//			FVector origin;
//			FVector boxExtend;
//			Actor->GetActorBounds(false, origin, boxExtend);
//			FVector sVector{ Location.X,Location.Y, boxExtend.Z * 2 + Actor->GetActorLocation().Z };
//			FVector eVector{ Location.X,Location.Y, Actor->GetActorLocation().Z - 1 };
//			FCollisionQueryParams traceParams;
//			traceParams.bTraceComplex = true;
//			world->LineTraceMultiByObjectType(traceHitResults, sVector, eVector, UEngineTypes::ConvertToObjectType(TraceType), traceParams);
//			for (size_t i = 0; i < traceHitResults.Num(); i++)
//			{
//				if (traceHitResults[i].GetActor() == Actor) {
//					if (traceHitResults[i].Location.Z > Z) {
//						Z = traceHitResults[i].Location.Z;
//					}
//				}
//			}
//		}
//		return Z < 1 && Z > -1 ? 0 : Z;
//	}
//	return -BIG_NUMBER;
//}

FSearchResult AGrid::AStarSearch(int32 startIndex, int32 endIndex, FAStarPreferences Preferences, bool bStopAtNeighborLocation, ESearchType SearchType, float NodeCostScale)
{
	SCOPE_CYCLE_COUNTER(STAT_ASTARSEARCH);

	if (startIndex == endIndex || (GridX * GridY) == 0)
		return FSearchResult();

	// No need to create this outside of the function. No one should use this.
	struct FANode
	{
	public:

		uint32 Closed : 1;
		int32 OpenSetIndex = -1;
		int32 parent = -1;
		int32 parentCount = 0;
		float TotalCost = 0.0f;
		float TraversalCost = 0.0f;
		float HeuristicCost = 0.0f;
		float NodeCostCount = 0.0f;
		float NodeCost;

		FANode()
			: Closed(false)
			, NodeCost(1.0f)
		{}
	};

	// Not to confuse with retracePath Function
	auto Retrace = [&](const int32 start, const int32 end, const TMap<int32, FANode>& StructData) -> FSearchResult
		{
			int32 current = end;
			FSearchResult Result;

			while (current != start)
			{
				if (current <= -1) {
					Result.ResultState = EAStarResultState::SearchFail;
					return Result;
				}

				FVector currentVector = Instances[current].Location;
				Result.PathResults.Insert(currentVector, 0);
				Result.PathIndexes.Insert(current, 0);
				Result.PathLength = Result.PathLength + 1;
				if (StructData.Find(current)) {
					float currentCost = StructData[current].NodeCost;
					Result.TotalNodeCost += currentCost;
					Result.PathCosts.Add(current, currentCost);
					Result.Parents.Add(current, StructData[current].parent);
					current = StructData[current].parent;
				}
				else {
					Result.ResultState = EAStarResultState::SearchFail;
					return Result;
				}
			}
			Result.ResultState = EAStarResultState::SearchSuccess;

			return Result;
		};

	const bool bFly = SearchType == ESearchType::Fly;

	FSearchResult			AStarResult;
	TMap<int32, FANode>		GridGraph;
	FNeighbors				stopAtNeighbor;

	TArray<int32> ObstacleIndexes;

	if (NodeCostScale < 1.0f)
		NodeCostScale = 1.0f;

	AStarResult.ResultState = EAStarResultState::SearchFail;

	TArray<FANode> fCostHeap;
	fCostHeap.AddDefaulted(1);
	fCostHeap[0].OpenSetIndex = startIndex;
	fCostHeap[0].parent = startIndex;

	auto Predicate = [&](const FANode& hIndex_A, const FANode& hIndex2_B) {return hIndex_A.TotalCost < hIndex2_B.TotalCost; };

	fCostHeap.Heapify(Predicate);

	if (startIndex < 0 || endIndex < 0 || startIndex > TotalGridSize || endIndex > TotalGridSize || !NodeBehavior(-1, endIndex, Preferences, SearchType).bAccess)
		return AStarResult;

	if (startIndex == endIndex) {
		AStarResult.ResultState = EAStarResultState::SearchSuccess;
		return AStarResult;
	}

	if (bStopAtNeighborLocation)
		stopAtNeighbor = calculateNeighborIndexes(endIndex);

	while (fCostHeap.Num() != 0) 
	{
		int32 CurrentIndex = fCostHeap[0].OpenSetIndex;

		if (bStopAtNeighborLocation ? stopAtNeighbor.Contains(CurrentIndex) : CurrentIndex == endIndex)
		{
			if (bStopAtNeighborLocation) {
				AStarResult = Retrace(startIndex, CurrentIndex, GridGraph);
			}
			else {
				AStarResult = Retrace(startIndex, CurrentIndex, GridGraph);
			}
			return AStarResult;
		}

		fCostHeap.HeapPopDiscard(Predicate);
		auto& CurrentNode = GridGraph.FindOrAdd(CurrentIndex);
		CurrentNode.Closed = true;

		FTileNeighborResult NeighborIndexes = GetNeighborIndexes(CurrentIndex, Preferences, SearchType);
		//if (NeighborIndexes.Num())
		//	return FAStarResult();

		if (Preferences.bRecordObstacleIndexes)
			ObstacleIndexes.Append(NeighborIndexes.ObstacleIndexes);

		for (const auto& Tile : NeighborIndexes.Neighbors)
		{
			auto& NextNode = GridGraph.FindOrAdd(Tile.Key);

			if (!NextNode.Closed)
			{
				float TraversalCost = (CurrentNode.TraversalCost + FIOctileDistance(Instances[CurrentIndex].Location, Instances[Tile.Key].Location)) + (bFly || Preferences.bOverrideNodeCostToOne ? 1.0f : (Tile.Value * NodeCostScale));

				auto PredicateIndex = [&](const FANode fCostH) {return fCostH.OpenSetIndex == Tile.Key; };

				if (TraversalCost < NextNode.TraversalCost || !fCostHeap.ContainsByPredicate(PredicateIndex))
				{
					NextNode.NodeCost = bFly || Preferences.bOverrideNodeCostToOne ? 1.0f : Tile.Value;
					NextNode.TraversalCost = TraversalCost;
					NextNode.parent = CurrentIndex;
					NextNode.NodeCostCount = CurrentNode.NodeCostCount + bFly || Preferences.bOverrideNodeCostToOne ? 1.0f : Tile.Value;
					NextNode.parentCount = CurrentNode.parentCount + 1;
					NextNode.OpenSetIndex = Tile.Key;
					NextNode.HeuristicCost = FIOctileDistance(Instances[Tile.Key].Location, Instances[endIndex].Location);	// NodePredicate
					NextNode.TotalCost = NextNode.TraversalCost + NextNode.HeuristicCost;	// NodePredicate

					if (!fCostHeap.ContainsByPredicate(PredicateIndex))
					{
						fCostHeap.HeapPush(NextNode, Predicate);
					}
				}
			}
		}
	}

	if (Preferences.bRecordObstacleIndexes)
	{
		for (const auto& idx : ObstacleIndexes)
		{
			AStarResult.ObstacleIndexes.AddUnique(idx);
		}
	}

	return AStarResult;
}

FSearchResult AGrid::PathSearchAtRange(int32 startIndex, int32 atRange, FAStarPreferences Preferences, ESearchType SearchType, float DefaultNodeCost)
{
	SCOPE_CYCLE_COUNTER(STAT_PathSearchAtRange);

	if (atRange == 0 || (GridX * GridY) == 0)
		return FSearchResult();

	// No need to create this outside of the function. No one should use this.
	struct FANode
	{
	public:

		uint32 Closed : 1;
		int32 OpenSetIndex = -1;
		int32 parent = -1;
		int32 parentCount = 0;
		float TotalCost = 0.0f;
		float TraversalCost = 0.0f;
		float HeuristicCost = 0.0f;
		float NodeCostCount = 0.0f;
		float NodeCost;

		FANode()
			: Closed(false)
			, NodeCost(1.0f)
		{}
	};

	FSearchResult Result;

	if (DefaultNodeCost < 1.0f)
		DefaultNodeCost = 1.0f;

	const bool bFly = SearchType == ESearchType::Fly;

	TMap<int32, float> closedSet;
	TMap<int32, float> openSet;
	TMap<int32, float> FinalSet;
	TMap<int32, FANode>	Node;

	TArray<int32> ObstacleIndexes;

	openSet.Add(startIndex, 1.0f);

	bool loop = true;
	bool pass = false;

	//int32 failedAttemptCounter = 0;

	while (loop) 
	{
		loop = false;
		pass = false;
		FTileNeighborResult NeighborIndexes_Pass1;
		if (closedSet.Num() == 0)
		{
			NeighborIndexes_Pass1 = GetNeighborIndexes(startIndex, Preferences, SearchType);
			if (NeighborIndexes_Pass1.Neighbors.Num() == 0)
				break;

			if (Preferences.bRecordObstacleIndexes)
				ObstacleIndexes.Append(NeighborIndexes_Pass1.ObstacleIndexes);
		}
		for (const auto& inx : closedSet.Num() == 0 ? NeighborIndexes_Pass1.Neighbors : closedSet)
		{
			FTileNeighborResult NeighborIndexes_Pass2;
			if (closedSet.Num() == 0)
			{
				NeighborIndexes_Pass2 = GetNeighborIndexes(startIndex, Preferences, SearchType);
				if (NeighborIndexes_Pass2.Neighbors.Num() == 0)
				{
					continue;
				}
				else
				{
					loop = true;
				}
			}
			else
			{
				NeighborIndexes_Pass2 = GetNeighborIndexes(inx.Key, Preferences, SearchType);
				if (NeighborIndexes_Pass2.Neighbors.Num() == 0)
				{
					continue;
				}
				else
				{
					loop = true;
				}
			}

			if (Preferences.bRecordObstacleIndexes)
				ObstacleIndexes.Append(NeighborIndexes_Pass2.ObstacleIndexes);

			for (const auto& loc : NeighborIndexes_Pass2.Neighbors)
			{
				if (loc.Key != startIndex) 
				{
					//failedAttemptCounter = 0;

					auto& NodeFragment = Node.FindOrAdd(loc.Key);

					float NodeCost = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ?
						NodeBehaviorBlueprint(inx.Key, loc.Key, Preferences, SearchType, GetNodeDirection(closedSet.Num() == 0 ? startIndex : inx.Key, loc.Key)).NodeCost
						: NodeBehavior(inx.Key, loc.Key, Preferences, SearchType, GetNodeDirection(closedSet.Num() == 0 ? startIndex : inx.Key, loc.Key)).NodeCost;
					NodeFragment.NodeCost = NodeCost;

					float tentativeCost = Node.FindOrAdd(inx.Key).NodeCostCount + NodeFragment.NodeCost;

					if (tentativeCost < NodeFragment.NodeCostCount || !openSet.Contains(loc.Key))
					{
						NodeFragment.parent = closedSet.Num() == 0 ? startIndex : inx.Key;
						NodeFragment.NodeCostCount = Preferences.bOverrideNodeCostToOne || bFly ? Node.FindOrAdd(NodeFragment.parent).NodeCostCount + 1
							: NodeCost + Node.FindOrAdd(NodeFragment.parent).NodeCostCount;
						loop = true;
						pass = true;
						if (NodeFragment.NodeCostCount <= (Preferences.bOverrideNodeCostToOne || bFly ? atRange : atRange * DefaultNodeCost))
						{
							NodeFragment.parentCount = Node.FindOrAdd(NodeFragment.parent).parentCount + 1;
							openSet.FindOrAdd(loc.Key) = (loc.Key, NodeCost);
							FinalSet.FindOrAdd(loc.Key) = (loc.Key, NodeCost);
						}
					}
					else if (!pass) 
					{
						loop = false;
					}
				}
			}
		}
		closedSet.Empty();
		closedSet = FinalSet;
		FinalSet.Empty();

		//failedAttemptCounter++;
		//if (failedAttemptCounter >= (GridX * GridY))
		//	return FSearchResult();
	}

	if (openSet.Num() > 1) 
	{
		Result.ResultState = EAStarResultState::SearchSuccess;
	}
	else 
	{
		Result.ResultState = EAStarResultState::SearchFail;
		return Result;
	}

	for (const auto& inx : openSet)
	{
		if (inx.Key != startIndex)
		{
			Result.PathResults.Add(Instances[inx.Key].Location);
			Result.Parents.Add(inx.Key, Node[inx.Key].parent);
			Result.PathIndexes.Add(inx.Key);
			Result.PathCosts.Add(inx.Key, inx.Value);
		}
	}

	if (Preferences.bRecordObstacleIndexes)
	{
		for (const auto& idx : ObstacleIndexes)
		{
			Result.ObstacleIndexes.AddUnique(idx);
		}
	}

	return Result;
}

FSearchResult AGrid::retracePath(int32 startIndex, int32 endIndex, FSearchResult StructData)
{
	SCOPE_CYCLE_COUNTER(STAT_ReTracePath);

	FSearchResult AResult;

	if (startIndex < 0 || endIndex < 0 || !StructData.PathIndexes.Contains(endIndex)) {
		AResult.ResultState = EAStarResultState::SearchFail;
		return AResult;
	}
	if (startIndex == endIndex) {
		AResult.ResultState = EAStarResultState::SearchSuccess;
		return AResult;
	}

	int32 currentIndex = endIndex;
	int32 debugCount = NULL;

	while (currentIndex != startIndex)
	{
		if (debugCount > TotalGridSize) { AResult.ResultState = EAStarResultState::InfiniteLoop; return AResult; }

		if (currentIndex <= -1) {
			AResult.ResultState = EAStarResultState::SearchFail;
			return AResult;
		}

		FVector currentVector = Instances[currentIndex].Location;
		AResult.PathResults.Insert(currentVector, 0);
		AResult.PathIndexes.Insert(currentIndex, 0);
		AResult.PathLength = AResult.PathLength + 1;
		if (StructData.PathCosts.Find(currentIndex)) {
			float currentCost = StructData.PathCosts[currentIndex];
			AResult.TotalNodeCost += currentCost;
			AResult.PathCosts.Add(currentIndex, currentCost);
		}
		if (StructData.Parents.Find(currentIndex)) {
			AResult.Parents.Add(currentIndex, *StructData.Parents.Find(currentIndex));
			currentIndex = *StructData.Parents.Find(currentIndex);
		}
		else {
			AResult.ResultState = EAStarResultState::SearchFail;
			return AResult;
		}
		debugCount++;
	}
	AResult.ResultState = EAStarResultState::SearchSuccess;

	/*if (GetTile(endIndex).Actor)
	{
		AResult.PathResults.RemoveAt(AResult.PathResults.Num() - 1);
		AResult.PathIndexes.RemoveAt(AResult.PathResults.Num() - 1);
		AResult.PathLength--;
		AResult.TotalNodeCost -= GetTile(endIndex).NodeProperty.NodeCost;
		AResult.PathCosts.Remove(endIndex);
		AResult.Parents.Remove(endIndex);
	}*/

	return AResult;
}

FNeighbors AGrid::calculateNeighborIndexes(int32 index)
{
	SCOPE_CYCLE_COUNTER(STAT_CalcNeighbor);

	/*
	* To Do : Fix out of bound results
	*/

	FNeighbors Neighbors;
	int32 hex;

	switch (gridType)
	{
	case EGridType::E_Square:

		Neighbors.EAST = GridY >= GridX ? (index + 1) % GridY == 0 ? -1 : index + 1 : (index + 1) % GridY == 0 ? -1 : index + 1;
		Neighbors.WEST = GridY >= GridX ? index % GridY == 0 ? -1 : index - 1 : index % GridY == 0 ? -1 : index - 1;
		Neighbors.SOUTH = GridY >= GridX ? index - GridY : index - GridY;
		Neighbors.NORTH = GridY >= GridX ? index + GridY : index + GridY;
		Neighbors.SOUTHEAST = GridY >= GridX ? (index + 1) % GridY == 0 ? -1 : (index - GridY) + 1 : (index + 1) % GridY == 0 ? -1 : (index - GridY) + 1;
		Neighbors.SOUTHWEST = GridY >= GridX ? index % GridY == 0 ? -1 : (index - GridY) - 1 : index % GridY == 0 ? -1 : (index - GridY) - 1;
		Neighbors.NORTHWEST = GridY >= GridX ? index % GridY == 0 ? -1 : (index + GridY) - 1 : index % GridY == 0 ? -1 : (index + GridY) - 1;
		Neighbors.NORTHEAST = GridY >= GridX ? (index + 1) % GridY == 0 ? -1 : (index + GridY) + 1 : (index + 1) % GridY == 0 ? -1 : (index + GridY) + 1;

		break;

	case EGridType::E_Hex:

		hex = index / GridX;
		if (hex % 2 != 0)
		{
			Neighbors.NORTHEAST = ((GridX - (-1)) + index) % GridX == 0 ? -1 : ((GridX - (-1)) + index);
			Neighbors.NORTHWEST = (index - (GridX - 1)) % GridX == 0 ? -1 : (index - (GridX - 1));
			Neighbors.SOUTHWEST = (index - GridX);
			Neighbors.SOUTHEAST = (GridX + index);
			Neighbors.NORTH = (index + 1) % GridX == 0 ? -1 : (index + 1);
			Neighbors.SOUTH = index % GridX == 0 ? -1 : (index - 1);
			Neighbors.EAST = -1;
			Neighbors.WEST = -1;
		}
		else {
			Neighbors.NORTHEAST = (GridX + index);
			Neighbors.NORTHWEST = (index - GridX);
			Neighbors.SOUTHWEST = /*(index - (GridX + 1))*/ index % GridX == 0 ? -1 : (index - (GridX + 1));
			Neighbors.SOUTHEAST = /*((GridX - (+1)) + index)*/ index % GridX == 0 ? -1 : ((GridX - (+1)) + index);
			Neighbors.NORTH = (index + 1) % GridX == 0 ? -1 : (index + 1);
			Neighbors.SOUTH = /*(index - 1)*/ index % GridX == 0 ? -1 : (index - 1);
			Neighbors.EAST = -1;
			Neighbors.WEST = -1;
		}

		break;

	default:
		break;
	}

	/* Temp fix for out of bound results */
	Neighbors.Filter(TotalGridSize);
	return Neighbors;
}

/**
*			+---+---+---+
*			| NW| N | NE|
*			+---+---+---+
*			| W |   | E |
*			+---+---+---+
*			| SW| S | SE|
*			+---+---+---+
*	EAST and WEST SQUARE Grid Only
*/
FTileNeighborResult AGrid::GetNeighborIndexes(int32 index, FAStarPreferences Preferences, ESearchType SearchType)
{
	SCOPE_CYCLE_COUNTER(STAT_GetNeighborIndexes);

	FTileNeighborResult Result;
	FNeighbors Neighbors = calculateNeighborIndexes(index);
	if (SearchType == ESearchType::Fly)
	{
		if (gridType == EGridType::E_Square && !Preferences.bDiagonal)
		{
			if (Neighbors.EAST != -1)
				Result.Neighbors.Add(Neighbors.EAST, 1.0);
			if (Neighbors.WEST != -1)
				Result.Neighbors.Add(Neighbors.WEST, 1.0);
			if (Neighbors.SOUTH != -1)
				Result.Neighbors.Add(Neighbors.SOUTH, 1.0);
			if (Neighbors.NORTH != -1)
				Result.Neighbors.Add(Neighbors.NORTH, 1.0);
			return Result;
		}
		else
		{
			Result.Neighbors = Neighbors.GetAllNodes();
			return Result;
		}
	}

	//TBitArray<FDefaultBitArrayAllocator> DiagonalBranch;
	//DiagonalBranch.Init(gridType == EGridType::E_Hex || Preferences.bDiagonal ? true : false, 8);

	FNodeProperty Access;

	if (Neighbors.EAST <= TotalGridSize && Neighbors.EAST >= 0 && gridType == EGridType::E_Square) 
	{
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.EAST, Preferences, SearchType, ENeighborDirection::EAST) : NodeBehavior(index, Neighbors.EAST, Preferences, SearchType, ENeighborDirection::EAST);
		if (Access.bAccess)
		{
			//DiagonalBranch[0] = true;
			Result.Neighbors.Add(Neighbors.EAST, Access.NodeCost);
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.EAST);
		}
	}
	if (Neighbors.WEST <= TotalGridSize && Neighbors.WEST >= 0 && gridType == EGridType::E_Square)
	{
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.WEST, Preferences, SearchType, ENeighborDirection::WEST) : NodeBehavior(index, Neighbors.WEST, Preferences, SearchType, ENeighborDirection::WEST);
		if (Access.bAccess)
		{
			//DiagonalBranch[1] = true;
			Result.Neighbors.Add(Neighbors.WEST, Access.NodeCost);
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.WEST);
		}
	}
	if (Neighbors.SOUTH <= TotalGridSize && Neighbors.SOUTH >= 0) 
	{
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.SOUTH, Preferences, SearchType, ENeighborDirection::SOUTH) : NodeBehavior(index, Neighbors.SOUTH, Preferences, SearchType, ENeighborDirection::SOUTH);
		if (Access.bAccess)
		{
			//DiagonalBranch[2] = true;
			Result.Neighbors.Add(Neighbors.SOUTH, Access.NodeCost);
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.SOUTH);
		}
	}
	if (Neighbors.NORTH <= TotalGridSize && Neighbors.NORTH >= 0) 
	{
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.NORTH, Preferences, SearchType, ENeighborDirection::NORTH) : NodeBehavior(index, Neighbors.NORTH, Preferences, SearchType, ENeighborDirection::NORTH);
		if (Access.bAccess)
		{
			//DiagonalBranch[3] = true;
			Result.Neighbors.Add(Neighbors.NORTH, Access.NodeCost);
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.NORTH);
		}
	}

	if (Neighbors.SOUTHEAST <= TotalGridSize && Neighbors.SOUTHEAST >= 0) 
	{
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.SOUTHEAST, Preferences, SearchType, ENeighborDirection::SOUTH_EAST) : NodeBehavior(index, Neighbors.SOUTHEAST, Preferences, SearchType, ENeighborDirection::SOUTH_EAST);
		if (/*(DiagonalBranch[0] && DiagonalBranch[2]) &&*/ Access.bAccess)
		{
			if (Preferences.bDiagonal && gridType == EGridType::E_Square)
			{
				//DiagonalBranch[4] = true;
				Result.Neighbors.Add(Neighbors.SOUTHEAST, Access.NodeCost);
			}
			else if (gridType == EGridType::E_Hex)
			{
				Result.Neighbors.Add(Neighbors.SOUTHEAST, Access.NodeCost);
			}
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.SOUTHEAST);
		}
	}
	if (Neighbors.SOUTHWEST <= TotalGridSize && Neighbors.SOUTHWEST >= 0) 
	{
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.SOUTHWEST, Preferences, SearchType, ENeighborDirection::SOUTH_WEST) : NodeBehavior(index, Neighbors.SOUTHWEST, Preferences, SearchType, ENeighborDirection::SOUTH_WEST);
		if (/*(DiagonalBranch[2] && DiagonalBranch[1]) &&*/ Access.bAccess)
		{
			if (Preferences.bDiagonal && gridType == EGridType::E_Square)
			{
				//DiagonalBranch[5] = true;
				Result.Neighbors.Add(Neighbors.SOUTHWEST, Access.NodeCost);
			}
			else if (gridType == EGridType::E_Hex)
			{
				Result.Neighbors.Add(Neighbors.SOUTHWEST, Access.NodeCost);
			}
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.SOUTHWEST);
		}
	}
	if (Neighbors.NORTHWEST <= TotalGridSize && Neighbors.NORTHWEST >= 0) 
	{
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.NORTHWEST, Preferences, SearchType, ENeighborDirection::NORTH_WEST) : NodeBehavior(index, Neighbors.NORTHWEST, Preferences, SearchType, ENeighborDirection::NORTH_WEST);
		if (/*(DiagonalBranch[1] && DiagonalBranch[3]) &&*/ Access.bAccess)
		{
			if (Preferences.bDiagonal && gridType == EGridType::E_Square)
			{
				//DiagonalBranch[6] = true;
				Result.Neighbors.Add(Neighbors.NORTHWEST, Access.NodeCost);
			}
			else if (gridType == EGridType::E_Hex)
			{
				Result.Neighbors.Add(Neighbors.NORTHWEST, Access.NodeCost);
			}
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.NORTHWEST);
		}
	}
	if (Neighbors.NORTHEAST <= TotalGridSize && Neighbors.NORTHEAST >= 0) 
	{
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.NORTHEAST, Preferences, SearchType, ENeighborDirection::NORTH_EAST) : NodeBehavior(index, Neighbors.NORTHEAST, Preferences, SearchType, ENeighborDirection::NORTH_EAST);
		if (/*(DiagonalBranch[3] && DiagonalBranch[0]) &&*/ Access.bAccess)
		{
			if (Preferences.bDiagonal && gridType == EGridType::E_Square)
			{
				//DiagonalBranch[7] = true;
				Result.Neighbors.Add(Neighbors.NORTHEAST, Access.NodeCost);
			}
			else if (gridType == EGridType::E_Hex)
			{
				Result.Neighbors.Add(Neighbors.NORTHEAST, Access.NodeCost);
			}
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.NORTHEAST);
		}
	}

	return Result;
}

ENeighborDirection AGrid::GetNodeDirection(int32 CurrentIndex, int32 NextIndex)
{
	SCOPE_CYCLE_COUNTER(STAT_GetNodeDirection);

	ENeighborDirection Direction = ENeighborDirection::None;

	FNeighbors Neighbors = calculateNeighborIndexes(CurrentIndex);

	if (Neighbors.EAST == NextIndex)
		Direction = ENeighborDirection::EAST;
	if (Neighbors.NORTH == NextIndex)
		Direction = ENeighborDirection::NORTH;
	if (Neighbors.NORTHEAST == NextIndex)
		Direction = ENeighborDirection::NORTH_EAST;
	if (Neighbors.NORTHWEST == NextIndex)
		Direction = ENeighborDirection::NORTH_WEST;
	if (Neighbors.SOUTH == NextIndex)
		Direction = ENeighborDirection::SOUTH;
	if (Neighbors.SOUTHEAST == NextIndex)
		Direction = ENeighborDirection::SOUTH_EAST;
	if (Neighbors.SOUTHWEST == NextIndex)
		Direction = ENeighborDirection::SOUTH_WEST;
	if (Neighbors.WEST == NextIndex)
		Direction = ENeighborDirection::WEST;

	return Direction;
}

FNodeProperty AGrid::NodeBehavior(int32 CurrentIndex, int32 NeighborIndex, FAStarPreferences Preferences, ESearchType SearchType, ENeighborDirection Direction)
{
	SCOPE_CYCLE_COUNTER(STAT_AccessNode);

	//FNodeProperty result;
	//result.bAccess = true;
	//result.NodeCost = Node.NodeProperty.NodeCost;

	return Instances[NeighborIndex].NodeProperty;
}

FNodeProperty AGrid::NodeBehaviorBlueprint_Implementation(int32 CurrentIndex, int32 NeighborIndex, FAStarPreferences Preferences, ESearchType SearchType, ENeighborDirection Direction)
{
	SCOPE_CYCLE_COUNTER(STAT_AccessNodeBlueprint);

	FNodeProperty result;
	result.bAccess = true;
	result.NodeCost = 1.0f;

	return result;
}

TArray<int32> AGrid::GetInstancesOverlappingBox(const FBox& Box) const
{
	/*if (Instances.Num() == 0)
	{
		UE_VLOG(GetOwner(), LogGridError, Log, TEXT("You are trying to access a tile on an empty grid."));
		return TArray<int32>();
	}*/

	FBox CellBound = GetTileBound();
	CellBound.Min = TileBound.Min * FVector(TileScale, 1.0f);
	CellBound.Max = TileBound.Max * FVector(TileScale, 1.0f);
	TArray<int32> indices;
	for (const auto& Instance : Instances)
	{
		CellBound = CellBound.MoveTo(Instance.Value.Location);
		if (Box.Intersect(CellBound))
		{
			indices.Add(Instance.Key);
		}
	}
	return indices;
}

TArray<int32> AGrid::GetInstancesOverlappingSphere(const FVector& Center, const float Radius) const
{
	/*if (Instances.Num() == 0)
	{
		UE_VLOG(GetOwner(), LogGridError, Log, TEXT("You are trying to access a tile on an empty grid."));
		return TArray<int32>();
	}*/

	FBox CellBound = GetTileBound();
	CellBound.Min = TileBound.Min * FVector(TileScale, 1.0f);
	CellBound.Max = TileBound.Max * FVector(TileScale, 1.0f);
	FSphere Sphere(Center, Radius);
	TArray<int32> indices;
	for (const auto& Instance : Instances)
	{
		CellBound = CellBound.MoveTo(Instance.Value.Location);
		if (FMath::SphereAABBIntersection(Sphere, CellBound))
		{
			indices.Add(Instance.Key);
		}
	}
	return indices;
}

int32 AGrid::GetTileIndex(FVector Point) const
{
	/*if (Instances.Num() == 0)
	{
		UE_VLOG(GetOwner(), LogGridError, Log, TEXT("You are trying to access a tile on an empty grid."));
		return -1;
	}*/

	FBox CellBound = GetTileBound();
	CellBound.Min = TileBound.Min * FVector(TileScale, 1.0f);
	CellBound.Max = TileBound.Max * FVector(TileScale, 1.0f);
	for (const auto& Instance : Instances)
	{
		CellBound = CellBound.MoveTo(Instance.Value.Location);
		if (CellBound.IsInside(Point))
		{
			return Instance.Key;
		}
	}
	return -1;
}

void AGrid::AddInstance(FVector Loc, FNodeProperty NodeProperty)
{
	Instances.Add(Instances.Num(), FGridNode(Loc, NodeProperty));
}

int32 AGrid::GetGridSize() const
{
	return Instances.Num();
}

void AGrid::ClearInstances()
{
	Instances.Empty();
}

bool AGrid::SetTileProperty(int32 index, FNodeProperty Property)
{
	if (!Instances.Contains(index))
		return false;

	Instances[index].NodeProperty = Property;
	return true;
}

bool AGrid::SetTilePropertyMap(TMap<int32, FNodeProperty> NodeProperties)
{
	if (NodeProperties.Num() == 0)
		return false;
	for (const auto& Node : NodeProperties)
	{
		if (Instances.Contains(Node.Key))
		{
			Instances[Node.Key].NodeProperty = Node.Value;
		}
	}
	return true;
}

void AGrid::RegisterActorToTile(int32 index, AActor* Actor)
{
	if (index < 0 || index > TotalGridSize || !Actor)
		return;

	Instances[index].NodeProperty.bAccess = false;
	Instances[index].Actor = Actor;
}

void AGrid::UnregisterActorFromTile(int32 index)
{
	if (index < 0 || index > TotalGridSize)
		return;

	Instances[index].NodeProperty.bAccess = true;
	Instances[index].Actor = nullptr;
}

FBox AGrid::GetTileBound() const
{
	return TileBound;

	/*switch (gridType)
	{
	case EGridType::E_Square:
	{
		return FBox(FVector(-100.00000000000000, -100.00000000000000, -200.00001525878906) * FVector(TileScale, 1.0f), FVector(100.00000000000000, 100.00003051757812, 0.0000000000000000) * FVector(TileScale, 1.0f));
	}
	break;
	case EGridType::E_Hex:
	{
		return FBox(FVector(-86.602546691894531, -100.00000000000000, -46.808815002441406) * FVector(TileScale, 1.0f), FVector(86.602546691894531, 100.00000000000000, 1.5258789062500000e-05) * FVector(TileScale, 1.0f));
	}
	break;
	default:
		break;
	}
	return FBox();*/
}
