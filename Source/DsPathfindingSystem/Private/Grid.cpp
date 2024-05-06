/*
* DsPathfindingSystem
* Plugin code
* Copyright (c) 2023 Davut Co�kun
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

AGrid::AGrid()
	: Super()
	, gridType(EGridType::E_Square)
	, GridX(0)
	, GridY(0)
	, TotalGridSize(0)
{
	TileScale.X = 1.0f;
	TileScale.Y = 1.0f;

	GridX = 0.0f;
	GridY = 0.0f;

	TileOffset.X = 0.0f;
	TileOffset.Y = 0.0f;

	scene = CreateDefaultSubobject<USceneComponent>(TEXT("USceneComponent"));
	scene->SetMobility(EComponentMobility::Static);
	RootComponent = scene;

	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
}

void AGrid::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
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

	TotalGridSize = (GridX * GridY) - 1;

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

FAStarResult AGrid::AStarSearch(int32 startIndex, int32 endIndex, FAStarPreferences Preferences, float NodeCostScale)
{
	SCOPE_CYCLE_COUNTER(STAT_ASTARSEARCH);

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
	auto Retrace = [&](const int32 start, const int32 end, const TMap<int32, FANode>& StructData) -> FAStarResult
		{
			int32 current = end;
			FAStarResult Result;

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

	FAStarResult			AStarResult;
	//FCriticalSection		mutex;
	TMap<int32, FANode>		GridGraph;
	FNeighbors				stopAtNeighbor;

	if (NodeCostScale < 1.0f)
		NodeCostScale = 1.0f;

	AStarResult.ResultState = EAStarResultState::SearchFail;

	TArray<FANode> fCostHeap;
	fCostHeap.AddDefaulted(1);
	fCostHeap[0].OpenSetIndex = startIndex;
	fCostHeap[0].parent = startIndex;

	auto Predicate = [&](const FANode& hIndex_A, const FANode& hIndex2_B) {return hIndex_A.TotalCost < hIndex2_B.TotalCost; };

	fCostHeap.Heapify(Predicate);

	if (startIndex < 0 || endIndex < 0 || startIndex > TotalGridSize || endIndex > TotalGridSize || !NodeBehavior(-1, endIndex).bAccess)
		return AStarResult;

	if (startIndex == endIndex) {
		AStarResult.ResultState = EAStarResultState::SearchSuccess;
		return AStarResult;
	}

	if (Preferences.bStopAtNeighborLocation)
		stopAtNeighbor = calculateNeighborIndexes(endIndex);

	while (fCostHeap.Num() != 0) {

		int32 CurrentIndex = fCostHeap[0].OpenSetIndex;

		if (Preferences.bStopAtNeighborLocation ? stopAtNeighbor.Contains(CurrentIndex) : CurrentIndex == endIndex)
		{
			if (Preferences.bStopAtNeighborLocation) {
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

		//mutex.Lock();
		for (auto& Tile : GetNeighborIndexes(CurrentIndex, Preferences)) {

			auto& NextNode = GridGraph.FindOrAdd(Tile.Key);

			if (!NextNode.Closed)
			{
				float TraversalCost = (CurrentNode.TraversalCost + FIOctileDistance(Instances[CurrentIndex].Location, Instances[Tile.Key].Location)) + (Preferences.bFly || Preferences.bOverrideNodeCostToOne ? 1.0f : (Tile.Value * NodeCostScale));

				auto PredicateIndex = [&](const FANode fCostH) {return fCostH.OpenSetIndex == Tile.Key; };

				if (TraversalCost < NextNode.TraversalCost || !fCostHeap.ContainsByPredicate(PredicateIndex))
				{
					NextNode.NodeCost = Preferences.bFly || Preferences.bOverrideNodeCostToOne ? 1.0f : Tile.Value;
					NextNode.TraversalCost = TraversalCost;
					NextNode.parent = CurrentIndex;
					NextNode.NodeCostCount = CurrentNode.NodeCostCount + Preferences.bFly || Preferences.bOverrideNodeCostToOne ? 1.0f : Tile.Value;
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
		//mutex.Unlock();
	}
	return AStarResult;
}

FPSARResult AGrid::PathSearchAtRange(int32 startIndex, int32 atRange, FAStarPreferences Preferences, float DefaultNodeCost)
{
	SCOPE_CYCLE_COUNTER(STAT_PathSearchAtRange);

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

	FPSARResult Result;

	//FCriticalSection mutex;

	if (DefaultNodeCost < 1.0f)
		DefaultNodeCost = 1.0f;

	TMap<int32, float> closedSet;
	TMap<int32, float> openSet;
	TMap<int32, float> FinalSet;
	TMap<int32, FANode>	Node;

	openSet.Add(startIndex, 1.0f);

	bool loop = true;
	bool pass = false;

	while (loop) {
		pass = false;
		for (auto& inx : closedSet.Num() == 0 ? GetNeighborIndexes(startIndex, Preferences) : closedSet) {

			for (auto& loc : closedSet.Num() == 0 ? GetNeighborIndexes(startIndex, Preferences) : GetNeighborIndexes(inx.Key, Preferences))
			{
				if (loc.Key != startIndex) {

					auto& NodeFragment = Node.FindOrAdd(loc.Key);

					float NodeCost = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ?
						NodeBehaviorBlueprint(inx.Key, loc.Key, GetNodeDirection(closedSet.Num() == 0 ? startIndex : inx.Key, loc.Key)).NodeCost
						: NodeBehavior(inx.Key, loc.Key, GetNodeDirection(closedSet.Num() == 0 ? startIndex : inx.Key, loc.Key)).NodeCost;
					NodeFragment.NodeCost = NodeCost;

					float tentativeCost = Node.FindOrAdd(inx.Key).NodeCostCount + NodeFragment.NodeCost;

					if (tentativeCost < NodeFragment.NodeCostCount || !openSet.Contains(loc.Key)) {

						NodeFragment.parent = closedSet.Num() == 0 ? startIndex : inx.Key;
						NodeFragment.NodeCostCount = Preferences.bOverrideNodeCostToOne || Preferences.bFly ? Node.FindOrAdd(NodeFragment.parent).NodeCostCount + 1
							: NodeCost + Node.FindOrAdd(NodeFragment.parent).NodeCostCount;
						loop = true;
						pass = true;
						if (NodeFragment.NodeCostCount <= (Preferences.bOverrideNodeCostToOne || Preferences.bFly ? atRange : atRange * DefaultNodeCost))
						{
							NodeFragment.parentCount = Node.FindOrAdd(NodeFragment.parent).parentCount + 1;
							openSet.FindOrAdd(loc.Key) = (loc.Key, NodeCost);
							FinalSet.FindOrAdd(loc.Key) = (loc.Key, NodeCost);
						}
					}
					else if (!pass) {
						loop = false;
					}
				}
			}
		}
		closedSet.Empty();
		closedSet = FinalSet;
		FinalSet.Empty();
	}

	if (openSet.Num() > 1) {
		Result.ResultState = EAStarResultState::SearchSuccess;
	}
	else {
		Result.ResultState = EAStarResultState::SearchFail;
		return Result;
	}

	for (auto& inx : openSet) {
		if (inx.Key != startIndex) {
			Result.PathResults.Add(Instances[inx.Key].Location);
			Result.Parents.Add(inx.Key, Node[inx.Key].parent);
			Result.PathIndexes.Add(inx.Key);
			Result.PathCosts.Add(inx.Key, inx.Value);
		}
	}

	return Result;
}

FPSARResult AGrid::retracePath(int32 startIndex, int32 endIndex, FPSARResult StructData)
{
	SCOPE_CYCLE_COUNTER(STAT_ReTracePath);

	FPSARResult AResult;

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
		if (debugCount > 7500) { AResult.ResultState = EAStarResultState::InfiniteLoop; return AResult; }

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
	return AResult;
}

FNeighbors AGrid::calculateNeighborIndexes(int32 index)
{
	SCOPE_CYCLE_COUNTER(STAT_CalcNeighbor);

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
TMap<int32, float> AGrid::GetNeighborIndexes(int32 index, FAStarPreferences Preferences)
{
	SCOPE_CYCLE_COUNTER(STAT_GetNeighborIndexes);

	FNeighbors Neighbors = calculateNeighborIndexes(index);
	if (Preferences.bFly)
		return Neighbors.GetAllNodes();

	TBitArray<FDefaultBitArrayAllocator> DiagonalBranch;
	DiagonalBranch.Init(gridType == EGridType::E_Hex || !Preferences.bDiagonal ? true : false, 8);

	TMap<int32, float> result;
	FNodeProperty Access;

	if (Neighbors.EAST <= TotalGridSize && Neighbors.EAST >= 0 && gridType == EGridType::E_Square) {
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.EAST, ENeighborDirection::E_EAST) : NodeBehavior(index, Neighbors.EAST, ENeighborDirection::E_EAST);
		if (Access.bAccess)
		{
			DiagonalBranch[0] = true;
			result.Add(Neighbors.EAST, Access.NodeCost);
		}
	}
	if (Neighbors.WEST <= TotalGridSize && Neighbors.WEST >= 0 && gridType == EGridType::E_Square) {
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.WEST, ENeighborDirection::E_WEST) : NodeBehavior(index, Neighbors.WEST, ENeighborDirection::E_WEST);
		if (Access.bAccess)
		{
			DiagonalBranch[1] = true;
			result.Add(Neighbors.WEST, Access.NodeCost);
		}
	}
	if (Neighbors.SOUTH <= TotalGridSize && Neighbors.SOUTH >= 0) {
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.SOUTH, ENeighborDirection::E_SOUTH) : NodeBehavior(index, Neighbors.SOUTH, ENeighborDirection::E_SOUTH);
		if (Access.bAccess)
		{
			DiagonalBranch[2] = true;
			result.Add(Neighbors.SOUTH, Access.NodeCost);
		}
	}
	if (Neighbors.NORTH <= TotalGridSize && Neighbors.NORTH >= 0) {
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.NORTH, ENeighborDirection::E_NORTH) : NodeBehavior(index, Neighbors.NORTH, ENeighborDirection::E_NORTH);
		if (Access.bAccess)
		{
			DiagonalBranch[3] = true;
			result.Add(Neighbors.NORTH, Access.NodeCost);
		}
	}
	if (Neighbors.SOUTHEAST <= TotalGridSize && Neighbors.SOUTHEAST >= 0) {
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.SOUTHEAST, ENeighborDirection::E_SOUTH_EAST) : NodeBehavior(index, Neighbors.SOUTHEAST, ENeighborDirection::E_SOUTH_EAST);
		if ((DiagonalBranch[0] && DiagonalBranch[2]) && Access.bAccess)
		{
			DiagonalBranch[4] = true;
			result.Add(Neighbors.SOUTHEAST, Access.NodeCost);
		}
	}

	if (Neighbors.SOUTHWEST <= TotalGridSize && Neighbors.SOUTHWEST >= 0) {
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.SOUTHWEST, ENeighborDirection::E_SOUTH_WEST) : NodeBehavior(index, Neighbors.SOUTHWEST, ENeighborDirection::E_SOUTH_WEST);
		if ((DiagonalBranch[2] && DiagonalBranch[1]) && Access.bAccess)
		{
			DiagonalBranch[5] = true;
			result.Add(Neighbors.SOUTHWEST, Access.NodeCost);
		}
	}
	if (Neighbors.NORTHWEST <= TotalGridSize && Neighbors.NORTHWEST >= 0) {
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.NORTHWEST, ENeighborDirection::E_NORTH_WEST) : NodeBehavior(index, Neighbors.NORTHWEST, ENeighborDirection::E_NORTH_WEST);
		if ((DiagonalBranch[1] && DiagonalBranch[3]) && Access.bAccess)
		{
			DiagonalBranch[6] = true;
			result.Add(Neighbors.NORTHWEST, Access.NodeCost);
		}
	}
	if (Neighbors.NORTHEAST <= TotalGridSize && Neighbors.NORTHEAST >= 0) {
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.NORTHEAST, ENeighborDirection::E_NORTH_EAST) : NodeBehavior(index, Neighbors.NORTHEAST, ENeighborDirection::E_NORTH_EAST);
		if ((DiagonalBranch[3] && DiagonalBranch[0]) && Access.bAccess)
		{
			DiagonalBranch[7] = true;
			result.Add(Neighbors.NORTHEAST, Access.NodeCost);
		}
	}

	return result;
}

ENeighborDirection AGrid::GetNodeDirection(int32 CurrentIndex, int32 NextIndex)
{
	SCOPE_CYCLE_COUNTER(STAT_GetNodeDirection);

	ENeighborDirection Direction = ENeighborDirection::E_NOWHERE;

	FNeighbors Neighbors = calculateNeighborIndexes(CurrentIndex);

	if (Neighbors.EAST == NextIndex)
		Direction = ENeighborDirection::E_EAST;
	if (Neighbors.NORTH == NextIndex)
		Direction = ENeighborDirection::E_NORTH;
	if (Neighbors.NORTHEAST == NextIndex)
		Direction = ENeighborDirection::E_NORTH_EAST;
	if (Neighbors.NORTHWEST == NextIndex)
		Direction = ENeighborDirection::E_NORTH_WEST;
	if (Neighbors.SOUTH == NextIndex)
		Direction = ENeighborDirection::E_SOUTH;
	if (Neighbors.SOUTHEAST == NextIndex)
		Direction = ENeighborDirection::E_SOUTH_EAST;
	if (Neighbors.SOUTHWEST == NextIndex)
		Direction = ENeighborDirection::E_SOUTH_WEST;
	if (Neighbors.WEST == NextIndex)
		Direction = ENeighborDirection::E_WEST;

	return Direction;
}

FNodeProperty AGrid::NodeBehavior(int32 CurrentIndex, int32 NeighborIndex, ENeighborDirection Direction)
{
	SCOPE_CYCLE_COUNTER(STAT_AccessNode);

	//FNodeProperty result;
	//result.bAccess = true;
	//result.NodeCost = 1.0f;

	return Instances[NeighborIndex].NodeProperty;
}

FNodeProperty AGrid::NodeBehaviorBlueprint_Implementation(int32 CurrentIndex, int32 NeighborIndex, ENeighborDirection Direction)
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

int32 AGrid::GetInstanceCount() const
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
