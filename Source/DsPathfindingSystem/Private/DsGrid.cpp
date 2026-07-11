/*
* DsPathfindingSystem
* Plugin code
* Copyright (c) 2023 Davut Co�kun
* All Rights Reserved.
*/

#include "DsGrid.h"

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

ADsGrid::ADsGrid()
	: Super()
	, GridType(EGridType::Square)
	, TileOrder(EGridTileOrder::RowMajor)
	, GridX(0)
	, GridY(0)
	, TileOffset(FVector2D(0.0, 0.0))
	, TileScale(FVector2D(1.0, 1.0))
	, bSquareGridDiagonalAllowed(false)
{
	Scene = CreateDefaultSubobject<USceneComponent>(TEXT("USceneComponent"));
	Scene->SetMobility(EComponentMobility::Static);
	RootComponent = Scene;
}

void ADsGrid::BeginPlay()
{
	Super::BeginPlay();
}

void ADsGrid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

bool ADsGrid::GenerateGridEx(EGridType InGridType, int32 InGridX, int32 InGridY, bool bIsSquareGridDiagonalAllowed, bool bUseCustomTileBounds, FBox CustomTileBounds, EGridTileOrder InTileOrder, TMap<int32, FNodeAttribute> NodeProperties, FVector2D InTileScale, FVector2D InTileOffset, bool bUseCustomGridLocation, FVector CustomGridLocation)
{
	ClearInstances();

	if (InGridX <= 0 || InGridY <= 0)
		return false;

	GridType = InGridType;
	TileOrder = InTileOrder;
	GridX = InGridX;
	GridY = InGridY;
	TileOffset = InTileOffset;
	TileScale = InTileScale;
	bSquareGridDiagonalAllowed = bIsSquareGridDiagonalAllowed;
	const FVector GridLoc = bUseCustomGridLocation ? CustomGridLocation : GetActorLocation();

	TileBound = bUseCustomTileBounds ? CustomTileBounds : GridType == EGridType::Square ? FBox(FVector(-100.00000000000000, -100.00000000000000, -200.00001525878906), FVector(100.00000000000000, 100.00003051757812, 0.0000000000000000)) : FBox(FVector(-86.602546691894531, -100.00000000000000, -46.808815002441406), FVector(86.602546691894531, 100.00000000000000, 1.5258789062500000e-05));

	const float boundX = TileBound.Max.X - TileBound.Min.X + TileOffset.X;
	const float boundY = TileBound.Max.Y - TileBound.Min.Y - TileOffset.Y;

	FVector NodeLoc = FVector::ZeroVector;

	for (int32 i = 0; i < (GridX * GridY); i++)
	{
		CreateNewInstance(i, NodeProperties.Contains(i) ? NodeProperties[i] : FNodeAttribute());
	}

	OnGridGenerated();

	return true;
}

bool ADsGrid::Resize(int32 NewSizeX, int32 NewSizeY)
{
	if (NewSizeX <= 0 || NewSizeY <= 0)
		return false;

	int32 Old_HorizontalSize = GridX;
	int32 Old_VerticalSize = GridY;

	GridX = NewSizeX;
	GridY = NewSizeY;

	//if (GridX < Old_HorizontalSize || GridY < Old_VerticalSize)
	{
		auto TempGridData = Instances;
		Instances.Empty();
		for (const auto& Elem : TempGridData)
		{
			auto& Tile = Elem.Value;

			const int32 x = TileOrder == EGridTileOrder::ColumnMajor ? Elem.Key / Old_VerticalSize : Elem.Key % Old_HorizontalSize;
			const int32 y = TileOrder == EGridTileOrder::ColumnMajor ? Elem.Key % Old_VerticalSize : Elem.Key / Old_HorizontalSize;

			if (x >= 0 && x < GridX && y >= 0 && y < GridY)
			{
				const int32 newIndex = TileOrder == EGridTileOrder::ColumnMajor ? (x * GridY) + y : (y * GridX) + x;

				if (newIndex >= 0 && newIndex < GridX * GridY)
					Instances.FindOrAdd(newIndex) = Elem.Value;
			}
			else
			{
				//DestroyTile(Elem.Key);
			}
		}
	}
	//else if (GridX > Old_HorizontalSize || GridY > Old_VerticalSize)
	{
		for (int32 ii = 0; ii < GridX * GridY; ii++)
		{
			if (Instances.Contains(ii))
				continue;
			CreateNewInstance(ii, FNodeAttribute(true, 1.0f, ETileType::Grass));
		}
	}

	OnResize(NewSizeX, NewSizeY);

	return true;
}

FSearchResult ADsGrid::AStarSearch(int32 StartIndex, int32 EndIndex, FAStarPreferences Preferences, bool bStopAtNeighborLocation, EGridHeuristicFunction HeuristicFunction) const
{
	SCOPE_CYCLE_COUNTER(STAT_ASTARSEARCH);

	if (!IsValidIndex(StartIndex) || !IsValidIndex(EndIndex))
		return FSearchResult();


	float NodeCostScale = 1.0f;

	if ((GridX * GridY) == 0)
		return FSearchResult();

	if (StartIndex == EndIndex)
	{
		FSearchResult Result;
		Result.EndPoint = EndIndex;
		Result.bStopAtNeighborLocation = bStopAtNeighborLocation;
		Result.ResultState = ESearchResult::AlreadyAtGoal;
		return Result;
	}
	else if (bStopAtNeighborLocation)
	{
		auto stopAtNeighbor = GetNeighborTilesAsArray(EndIndex, Preferences.bBlockBorder);
		if (stopAtNeighbor.Contains(StartIndex))
		{
			FSearchResult Result;
			Result.EndPoint = EndIndex;
			Result.bStopAtNeighborLocation = bStopAtNeighborLocation;
			Result.ResultState = ESearchResult::AlreadyAtGoal;
			return Result;
		}
	}

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

			Result.EndPoint = EndIndex;
			Result.bStopAtNeighborLocation = bStopAtNeighborLocation;

			while (current != start)
			{
				if (current <= -1) {
					Result.ResultState = ESearchResult::SearchFail;
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
					Result.ResultState = ESearchResult::SearchFail;
					return Result;
				}
			}
			Result.ResultState = ESearchResult::SearchSuccess;

			return Result;
		};

	FSearchResult			AStarResult;
	TMap<int32, FANode>		GridGraph;
	TArray<int32>			stopAtNeighbor;

	TArray<int32> ObstacleIndexes;

	AStarResult.EndPoint = EndIndex;
	AStarResult.bStopAtNeighborLocation = bStopAtNeighborLocation;

	if (NodeCostScale < 1.0f)
		NodeCostScale = 1.0f;

	AStarResult.ResultState = ESearchResult::SearchFail;

	TArray<FANode> fCostHeap;
	fCostHeap.AddDefaulted(1);
	fCostHeap[0].OpenSetIndex = StartIndex;
	fCostHeap[0].parent = StartIndex;

	auto Predicate = [&](const FANode& hIndex_A, const FANode& hIndex2_B) {return hIndex_A.TotalCost < hIndex2_B.TotalCost; };

	fCostHeap.Heapify(Predicate);

	if (StartIndex < 0 || EndIndex < 0 || StartIndex >= (GridX * GridY) || EndIndex >= (GridX * GridY) || (!bStopAtNeighborLocation && !NodeBehavior(-1, EndIndex, EndIndex, Preferences).bAccess))
		return AStarResult;

	if (bStopAtNeighborLocation)
		stopAtNeighbor = GetNeighborTilesAsArray(EndIndex, Preferences.bBlockBorder);

	while (fCostHeap.Num() != 0)
	{
		int32 CurrentIndex = fCostHeap[0].OpenSetIndex;

		if (bStopAtNeighborLocation ? stopAtNeighbor.Contains(CurrentIndex) : CurrentIndex == EndIndex)
		{
			if (bStopAtNeighborLocation) {
				AStarResult = Retrace(StartIndex, CurrentIndex, GridGraph);
			}
			else {
				AStarResult = Retrace(StartIndex, CurrentIndex, GridGraph);
			}
			return AStarResult;
		}

		fCostHeap.HeapPopDiscard(Predicate);
		auto& CurrentNode = GridGraph.FindOrAdd(CurrentIndex);
		CurrentNode.Closed = true;

		FTileNeighborResult NeighborIndexes = GetNeighborIndexes(CurrentIndex, EndIndex, Preferences);

		if (Preferences.bRecordObstacleIndexes)
			ObstacleIndexes.Append(NeighborIndexes.ObstacleIndexes);

		for (const auto& Tile : NeighborIndexes.Neighbors)
		{
			auto& NextNode = GridGraph.FindOrAdd(Tile.Key);

			if (!NextNode.Closed)
			{
				float TraversalCost = (CurrentNode.TraversalCost + GetHeuristic(HeuristicFunction, Instances[CurrentIndex].Location, Instances[Tile.Key].Location)) + (Preferences.bOverrideNodeCostToOne ? 1.0f : ((Tile.Value.NodeCost * Tile.Value.NodeCostScale) * NodeCostScale));

				auto PredicateIndex = [&](const FANode fCostH) {return fCostH.OpenSetIndex == Tile.Key; };

				if (TraversalCost < NextNode.TraversalCost || !fCostHeap.ContainsByPredicate(PredicateIndex))
				{
					NextNode.NodeCost = Preferences.bOverrideNodeCostToOne ? 1.0f : Tile.Value.NodeCost;
					NextNode.TraversalCost = TraversalCost;
					NextNode.parent = CurrentIndex;
					NextNode.NodeCostCount = CurrentNode.NodeCostCount + (Preferences.bOverrideNodeCostToOne ? 1.0f : Tile.Value.NodeCost);
					NextNode.parentCount = CurrentNode.parentCount + 1;
					NextNode.OpenSetIndex = Tile.Key;
					NextNode.HeuristicCost = GetHeuristic(HeuristicFunction, Instances[Tile.Key].Location, Instances[EndIndex].Location);	// NodePredicate
					NextNode.TotalCost = NextNode.TraversalCost + NextNode.HeuristicCost;	// NodePredicate

					if (Preferences.TotalNodeCostLimit >= 0 && NextNode.TotalCost > Preferences.TotalNodeCostLimit)
					{
						if (Preferences.bRecordObstacleIndexes)
						{
							for (const auto& idx : ObstacleIndexes)
							{
								AStarResult.ObstacleIndexes.AddUnique(idx);
							}
						}

						AStarResult = Retrace(StartIndex, CurrentIndex, GridGraph);

						if (Preferences.bFailIfTotalNodeCostExceeded)
						{
							AStarResult.ResultState = ESearchResult::SearchFail;
						}

						return AStarResult;
					}

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

#if 0
// Generated by gemini AI
FSearchResult ADsGrid::PathSearchAtRange(int32 StartIndex, int32 AtRange, FAStarPreferences Preferences) const
{
	SCOPE_CYCLE_COUNTER(STAT_PathSearchAtRange);

	// 1. Validation
	if (AtRange <= 0 || (GridX * GridY) <= 0)
	{
		return FSearchResult();
	}

	// 2. Setup Data Structures
	FSearchResult Result;

	// CostSoFar: Stores the best cost found to reach a specific Index. 
	// This acts as our "Visisted" set. If we find a path to an Index that is cheaper, we update this.
	TMap<int32, float> CostSoFar;

	// CameFrom: Stores the parent Index for path reconstruction.
	TMap<int32, int32> CameFrom;

	// Node wrapper for the Priority Queue
	struct FSearchNode
	{
		int32 Index;
		float Cost;

		// Operator for Min-Heap (lowest cost pops first)
		// Note: UE Heaps are Max-Heaps by default, so we invert logic here or use a predicate.
		// We will use a predicate in HeapPush/Pop for clarity.

		//// Comparison for min-heap
		//bool operator>(const FSearchNode& Other) const
		//{
		//	return Cost > Other.Cost;
		//}
		//bool operator<(const FSearchNode& Other) const
		//{
		//	return Cost < Other.Cost;
		//}
	};

	// The OpenSet (Priority Queue)
	TArray<FSearchNode> OpenSet;

	// 3. Initialize Start Node
	CostSoFar.Add(StartIndex, 0.0f);
	CameFrom.Add(StartIndex, -1);

	// Heap Predicate: Returns true if A is "less priority" than B. 
	// For Dijkstra, we want smallest Cost at the top, so we return true if A.Cost > B.Cost.
	auto HeapPred = [](const FSearchNode& A, const FSearchNode& B) {
		return A.Cost > B.Cost;
		};

	OpenSet.HeapPush({ StartIndex, 0.0f }, HeapPred);

	// Obstacle storage
	TArray<int32> RecordedObstacles;
	bool bLimitExceeded = false;
	float RangeLimit = (float)AtRange;

	// 4. Main Dijkstra Loop
	while (OpenSet.Num() > 0)
	{
		// Get node with lowest cost
		FSearchNode Current = OpenSet[0];
		OpenSet.HeapPop(Current, HeapPred, false); // Pop and maintain heap property

		// Optimization: If we pulled a stale node from the heap (we found a cheaper way to this Index later), skip it.
		if (Current.Cost > CostSoFar[Current.Index])
		{
			continue;
		}

		// Get Neighbors
		FTileNeighborResult NeighborsResult = GetNeighborIndexes(Current.Index, -1, Preferences);

		// Handle Obstacles if requested
		if (Preferences.bRecordObstacleIndexes && NeighborsResult.ObstacleIndexes.Num() > 0)
		{
			RecordedObstacles.Append(NeighborsResult.ObstacleIndexes);
		}

		// Process Neighbors
		for (const auto& NeighborEntry : NeighborsResult.Neighbors)
		{
			int32 NeighborIndex = NeighborEntry.Key;

			// Skip start node self-reference
			if (NeighborIndex == StartIndex) continue;

			const FTileNeighborCost& NeighborData = NeighborEntry.Value;

			// Calculate new cost to get here
			// Note: If Preferences.bOverrideNodeCostToOne is true, we treat edge weight as 1.
			float AddedCost = Preferences.bOverrideNodeCostToOne ? 1.0f : (NeighborData.NodeCost * NeighborData.NodeCostScale);
			float NewCost = Current.Cost + AddedCost;

			// 4a. Check Limits
			if (Preferences.TotalNodeCostLimit >= 0 && NewCost > Preferences.TotalNodeCostLimit)
			{
				if (Preferences.bFailIfTotalNodeCostExceeded)
				{
					bLimitExceeded = true;
					OpenSet.Empty(); // Force break outer loop
					break;
				}
				continue; // Just skip this node
			}

			// 4b. Check Range
			// If the cost exceeds our search range, don't add to queue
			if (NewCost > RangeLimit)
			{
				continue;
			}

			// 4c. Relaxation Step
			// If we found a cheaper path to this neighbor (or haven't visited it yet)
			float* ExistingCost = CostSoFar.Find(NeighborIndex);
			if (!ExistingCost || NewCost < *ExistingCost)
			{
				CostSoFar.Add(NeighborIndex, NewCost);
				CameFrom.Add(NeighborIndex, Current.Index);
				OpenSet.HeapPush({ NeighborIndex, NewCost }, HeapPred);
			}
		}

		if (bLimitExceeded) break;
	}

	// 5. Construct Result
	if (bLimitExceeded)
	{
		Result.ResultState = ESearchResult::SearchFail;
		return Result;
	}

	// Populate FSearchResult based on the Visited data (CostSoFar)
	// If CostSoFar has more than just the start Index, we found something.
	if (CostSoFar.Num() > 1)
	{
		Result.ResultState = ESearchResult::SearchSuccess;

		for (const auto& It : CostSoFar)
		{
			int32 Index = It.Key;
			float Cost = It.Value;

			if (Index == StartIndex) continue;

			// Add path details
			Result.PathIndexes.Add(Index);
			Result.PathCosts.Add(Index, Cost);

			// Reconstruct location (Assuming you have an Instances array)
			if (Instances.Contains(Index))
			{
				Result.PathResults.Add(Instances[Index].Location);
			}

			// Add Parent
			if (CameFrom.Contains(Index))
			{
				Result.Parents.Add(Index, CameFrom[Index]);
			}
		}
	}
	else
	{
		Result.ResultState = ESearchResult::SearchFail;
	}

	// 6. Add Obstacles (Unique)
	if (Preferences.bRecordObstacleIndexes)
	{
		for (int32 ObsIndex : RecordedObstacles)
		{
			Result.ObstacleIndexes.AddUnique(ObsIndex);
		}
	}

	return Result;
}

FSearchResult ADsGrid::RetracePath(int32 StartIndex, int32 EndIndex, FSearchResult StructData, bool bStopAtNeighborLocation) const
{
	SCOPE_CYCLE_COUNTER(STAT_ReTracePath);

	FSearchResult Result;

	// 1. Basic Validation
	// If start/end are invalid, or if we aren't stopping at a neighbor AND the exact end wasn't found... fail.
	if (StartIndex < 0 || EndIndex < 0 || (!bStopAtNeighborLocation && !StructData.Parents.Contains(EndIndex)))
	{
		Result.ResultState = ESearchResult::SearchFail;
		return Result;
	}

	// 2. Trivial Case: Already at location
	if (StartIndex == EndIndex)
	{
		Result.EndPoint = EndIndex;
		Result.bStopAtNeighborLocation = bStopAtNeighborLocation;
		Result.ResultState = ESearchResult::AlreadyAtGoal;
		return Result;
	}

	// 3. Helper Lambda: Reconstructs the path for a specific TargetIndex
	// This is now O(N) because we Add() and Reverse(), instead of Insert(0)
	auto BuildPath = [&](int32 TargetIndex) -> FSearchResult
		{
			FSearchResult LocalResult;
			LocalResult.EndPoint = TargetIndex;
			LocalResult.bStopAtNeighborLocation = bStopAtNeighborLocation;

			int32 CurrentIndex = TargetIndex;
			int32 SafetyCount = 0;
			const int32 MaxLoop = GridX * GridY;

			while (CurrentIndex != StartIndex)
			{
				// Infinite loop protection
				if (SafetyCount++ > MaxLoop)
				{
					LocalResult.ResultState = ESearchResult::InfiniteLoop;
					return LocalResult;
				}

				// Validation check
				if (CurrentIndex < 0 || !Instances.Contains(CurrentIndex))
				{
					LocalResult.ResultState = ESearchResult::SearchFail;
					return LocalResult;
				}

				// 1. Add Data (Reverse order initially)
				LocalResult.PathResults.Add(Instances[CurrentIndex].Location);
				LocalResult.PathIndexes.Add(CurrentIndex);

				// 2. Accumulate Cost
				if (const float* CostPtr = StructData.PathCosts.Find(CurrentIndex))
				{
					float StepCost = *CostPtr;
					LocalResult.TotalNodeCost += StepCost;
					LocalResult.PathCosts.Add(CurrentIndex, StepCost);
				}

				// 3. Move to Parent
				if (const int32* ParentPtr = StructData.Parents.Find(CurrentIndex))
				{
					CurrentIndex = *ParentPtr;
				}
				else
				{
					// Logic break: We are not at start, but have no parent.
					LocalResult.ResultState = ESearchResult::SearchFail;
					return LocalResult;
				}
			}

			// Add the start node logic (usually start node has 0 cost, but needs to be in the path array)
			/*if (Instances.Contains(startIndex))
			{
				LocalResult.PathResults.Add(Instances[startIndex].Location);
				LocalResult.PathIndexes.Add(startIndex);
			}*/

			// 4. Reverse arrays to correct order (Start -> End)
			Algo::Reverse(LocalResult.PathResults);
			Algo::Reverse(LocalResult.PathIndexes);
			// PathCosts is a Map, so order doesn't matter there. 
			// If you strictly need ordered costs, you should use an Array<float> instead of a TMap.

			LocalResult.PathLength = LocalResult.PathIndexes.Num();
			LocalResult.ResultState = ESearchResult::SearchSuccess;

			return LocalResult;
		};


	// 4. Handling "Stop At Neighbor"
	if (bStopAtNeighborLocation)
	{
		TArray<int32> Neighbors = GetNeighborTilesAsArray(EndIndex);

		// Fast Check: If Start is already a neighbor of End
		if (Neighbors.Contains(StartIndex))
		{
			Result.EndPoint = EndIndex;
			Result.bStopAtNeighborLocation = true;
			Result.ResultState = ESearchResult::AlreadyAtGoal;
			return Result;
		}

		int32 BestNeighborIndex = -1;
		float BestCost = MAX_flt;

		// Find the best neighbor (Cheaply)
		// We do NOT build the full path here. We just walk parents to check costs.
		for (int32 NeighborIdx : Neighbors)
		{
			// Skip invalid neighbors
			if (!StructData.Parents.Contains(NeighborIdx) /*|| NeighborIndexesToFilter.Contains(NeighborIdx)*/)
			{
				continue;
			}

			float CurrentPathCost = 0.f;
			int32 TraceIdx = NeighborIdx;
			int32 Safety = 0;
			bool bPathValid = true;

			// Light-weight retrace to calculate score
			while (TraceIdx != StartIndex)
			{
				if (Safety++ > (GridX * GridY) || !StructData.Parents.Contains(TraceIdx))
				{
					bPathValid = false;
					break;
				}

				if (const float* C = StructData.PathCosts.Find(TraceIdx))
				{
					CurrentPathCost += *C;
				}

				TraceIdx = StructData.Parents[TraceIdx];
			}

			if (bPathValid && CurrentPathCost < BestCost)
			{
				BestCost = CurrentPathCost;
				BestNeighborIndex = NeighborIdx;
			}
		}

		// If we found a valid neighbor, build the full path for ONLY that one
		if (BestNeighborIndex != -1)
		{
			return BuildPath(BestNeighborIndex);
		}
		else
		{
			// Logic: Requested stop at neighbor, but no neighbors were reachable
			Result.ResultState = ESearchResult::SearchFail;
			return Result;
		}
	}

	// 5. Standard Path (Exact End Index)
	return BuildPath(EndIndex);
}

#else

FSearchResult ADsGrid::PathSearchAtRange(int32 StartIndex, int32 AtRange, FAStarPreferences Preferences) const
{
	SCOPE_CYCLE_COUNTER(STAT_PathSearchAtRange);

	if (!IsValidIndex(StartIndex))
		return FSearchResult();

	if (AtRange == 0 || (GridX * GridY) <= 0)
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

	float DefaultNodeCost = 1.0f;

	TMap<int32, FTileNeighborCost> closedSet;
	TMap<int32, FTileNeighborCost> openSet;
	TMap<int32, FTileNeighborCost> FinalSet;
	TMap<int32, FANode>	Node;

	TArray<int32> ObstacleIndexes;

	openSet.Add(StartIndex, 1.0f);

	bool loop = true;
	bool pass = false;
	bool wayout = false;

	bool bForceFail = false;

	while (loop)
	{
		loop = false;
		pass = false;
		wayout = false;
		FTileNeighborResult NeighborIndexes_Pass1;
		if (closedSet.Num() == 0)
		{
			NeighborIndexes_Pass1 = GetNeighborIndexes(StartIndex, -1, Preferences);
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
				NeighborIndexes_Pass2 = GetNeighborIndexes(StartIndex, -1, Preferences);
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
				NeighborIndexes_Pass2 = GetNeighborIndexes(inx.Key, -1, Preferences);
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
				if (loc.Key != StartIndex)
				{
					auto& NodeFragment = Node.FindOrAdd(loc.Key);

					float NodeCost = loc.Value.NodeCost;
					NodeFragment.NodeCost = NodeCost;

					float tentativeCost = Node.FindOrAdd(inx.Key).NodeCostCount + (NodeFragment.NodeCost * loc.Value.NodeCostScale);

					if (tentativeCost < NodeFragment.NodeCostCount || !openSet.Contains(loc.Key))
					{
						NodeFragment.parent = closedSet.Num() == 0 ? StartIndex : inx.Key;
						NodeFragment.NodeCostCount = Preferences.bOverrideNodeCostToOne ? Node.FindOrAdd(NodeFragment.parent).NodeCostCount + 1
							: (NodeCost * loc.Value.NodeCostScale) + Node.FindOrAdd(NodeFragment.parent).NodeCostCount;
						loop = true;
						pass = true;

						bool bSkip = false;
						if (Preferences.TotalNodeCostLimit >= 0 && NodeFragment.NodeCostCount > Preferences.TotalNodeCostLimit)
						{
							bSkip = true;

							if (Preferences.bFailIfTotalNodeCostExceeded)
							{
								bForceFail = true;
								loop = false;
								pass = false;
								wayout = false;
							}
						}

						if (NodeFragment.NodeCostCount <= (Preferences.bOverrideNodeCostToOne ? AtRange : AtRange * DefaultNodeCost) && !bSkip)
						{
							wayout = true;
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

		if (!wayout)
			break;
	}

	if (openSet.Num() > 1)
	{
		Result.ResultState = ESearchResult::SearchSuccess;
	}
	else
	{
		Result.ResultState = ESearchResult::SearchFail;
		return Result;
	}

	for (const auto& inx : openSet)
	{
		if (inx.Key != StartIndex)
		{
			Result.PathResults.Add(Instances[inx.Key].Location);
			Result.Parents.Add(inx.Key, Node[inx.Key].parent);
			Result.PathIndexes.Add(inx.Key);
			Result.PathCosts.Add(inx.Key, inx.Value.NodeCost);
		}
	}

	if (Preferences.bRecordObstacleIndexes)
	{
		for (const auto& idx : ObstacleIndexes)
		{
			Result.ObstacleIndexes.AddUnique(idx);
		}
	}

	if (bForceFail)
	{
		Result.ResultState = ESearchResult::SearchFail;
	}

	return Result;
}

FSearchResult ADsGrid::RetracePath(int32 StartIndex, int32 EndIndex, FSearchResult StructData, bool bStopAtNeighborLocation) const
{
	SCOPE_CYCLE_COUNTER(STAT_ReTracePath);

	FSearchResult AResult;

	if (!IsValidIndex(StartIndex) || !IsValidIndex(EndIndex) || (!bStopAtNeighborLocation && !StructData.PathIndexes.Contains(EndIndex))) {
		AResult.ResultState = ESearchResult::SearchFail;
		return AResult;
	}

	if (StartIndex == EndIndex)
	{
		FSearchResult Result;
		Result.EndPoint = EndIndex;
		Result.bStopAtNeighborLocation = bStopAtNeighborLocation;
		Result.ResultState = ESearchResult::AlreadyAtGoal;
		return Result;
	}
	else if (bStopAtNeighborLocation)
	{
		auto stopAtNeighbor = GetNeighborTilesAsArray(EndIndex);
		if (stopAtNeighbor.Contains(StartIndex))
		{
			FSearchResult Result;
			Result.EndPoint = EndIndex;
			Result.bStopAtNeighborLocation = bStopAtNeighborLocation;
			Result.ResultState = ESearchResult::AlreadyAtGoal;
			return Result;
		}
	}

	auto Retrace = [&](const int32 Start, const int32 End, const FSearchResult& Data) -> FSearchResult
		{
			FSearchResult SearchResult;
			int32 currentIndex = End;
			int32 debugCount = NULL;

			SearchResult.EndPoint = End;
			SearchResult.bStopAtNeighborLocation = bStopAtNeighborLocation;

			while (currentIndex != Start)
			{
				if (debugCount >= (GridX * GridY)) { SearchResult.ResultState = ESearchResult::InfiniteLoop; return SearchResult; }

				if (currentIndex <= -1) {
					SearchResult.ResultState = ESearchResult::SearchFail;
					return SearchResult;
				}

				FVector currentVector = Instances[currentIndex].Location;
				SearchResult.PathResults.Insert(currentVector, 0);
				SearchResult.PathIndexes.Insert(currentIndex, 0);
				SearchResult.PathLength = SearchResult.PathLength + 1;
				if (Data.PathCosts.Find(currentIndex)) {
					float currentCost = Data.PathCosts[currentIndex];
					SearchResult.TotalNodeCost += currentCost;
					SearchResult.PathCosts.Add(currentIndex, currentCost);
				}
				if (Data.Parents.Find(currentIndex)) {
					SearchResult.Parents.Add(currentIndex, *Data.Parents.Find(currentIndex));
					currentIndex = *Data.Parents.Find(currentIndex);
				}
				else {
					SearchResult.ResultState = ESearchResult::SearchFail;
					return SearchResult;
				}
				debugCount++;
			}

			SearchResult.ResultState = ESearchResult::SearchSuccess;
			return SearchResult;
		};

	if (bStopAtNeighborLocation)
	{
		TArray<int32> EndIndexes = GetNeighborTilesAsArray(EndIndex);
		TArray<FSearchResult> SearchResults;
		for (const auto& End : EndIndexes)
		{
			if (!StructData.PathIndexes.Contains(End) /*|| NeighborIndexesToFilter.Contains(End)*/)
				continue;
			FSearchResult SearchResult = Retrace(StartIndex, End, StructData);
			if (SearchResult.ResultState == ESearchResult::SearchSuccess)
				SearchResults.Add(SearchResult);
		}

		float PathCost = MAX_flt; //TNumericLimits<float>::Max;
		FSearchResult SearchResult;
		for (const auto& Result : SearchResults)
		{
			if (Result.TotalNodeCost <= PathCost)
			{
				PathCost = Result.TotalNodeCost;
				SearchResult = Result;
			}
		}

		return SearchResult;
	}

	return Retrace(StartIndex, EndIndex, StructData);
}
#endif

FNeighbors ADsGrid::GetNeighborTiles(int32 Index, bool bBlockBorder) const
{
	if (!IsValidIndex(Index))
		return FNeighbors();

	//SCOPE_CYCLE_COUNTER(STAT_CalcNeighbor);

	FNeighbors Neighbors;

	// North/South coordinate
	int32 AxisNS = 0;
	// East/West coordinate
	int32 AxisEW = 0;

	if (TileOrder == EGridTileOrder::RowMajor)
	{
		AxisNS = Index % GridX;
		AxisEW = Index / GridX;
	}
	else
	{
		AxisEW = Index % GridY;
		AxisNS = Index / GridY;
	}

	auto GetNeighborIndex = [&](int32 dN, int32 dE) -> int32
		{
			int32 nN = AxisNS + dN;
			int32 nE = AxisEW + dE;

			if (bBlockBorder)
			{
				if (nN < 0 || nN >= GridX || nE < 0 || nE >= GridY)
					return -1;
			}
			else
			{
				nN = (nN % GridX + GridX) % GridX;
				nE = (nE % GridY + GridY) % GridY;
			}

			return (TileOrder == EGridTileOrder::RowMajor) ? (nE * GridX + nN) : (nN * GridY + nE);
		};

	if (GridType == EGridType::Square)
	{
		Neighbors.NORTH = GetNeighborIndex(1, 0);
		Neighbors.SOUTH = GetNeighborIndex(-1, 0);
		Neighbors.EAST = GetNeighborIndex(0, 1);
		Neighbors.WEST = GetNeighborIndex(0, -1);
		if (bSquareGridDiagonalAllowed)
		{
			Neighbors.NORTHEAST = GetNeighborIndex(1, 1);
			Neighbors.NORTHWEST = GetNeighborIndex(1, -1);
			Neighbors.SOUTHEAST = GetNeighborIndex(-1, 1);
			Neighbors.SOUTHWEST = GetNeighborIndex(-1, -1);
		}
	}
	else if (GridType == EGridType::Hex)
	{
		Neighbors.EAST = -1;
		Neighbors.WEST = -1;

		Neighbors.NORTH = GetNeighborIndex(1, 0);
		Neighbors.SOUTH = GetNeighborIndex(-1, 0);

		if (AxisEW % 2 != 0)
		{
			Neighbors.NORTHEAST = GetNeighborIndex(1, 1);
			Neighbors.NORTHWEST = GetNeighborIndex(1, -1);
			Neighbors.SOUTHEAST = GetNeighborIndex(0, 1);
			Neighbors.SOUTHWEST = GetNeighborIndex(0, -1);
		}
		else
		{
			Neighbors.NORTHEAST = GetNeighborIndex(0, 1);
			Neighbors.NORTHWEST = GetNeighborIndex(0, -1);
			Neighbors.SOUTHEAST = GetNeighborIndex(-1, 1);
			Neighbors.SOUTHWEST = GetNeighborIndex(-1, -1);
		}
	}

	// Filters out any invalid or out-of-bounds indexes
	Neighbors.Filter((GridX * GridY));

	return Neighbors;
}

TArray<int32> ADsGrid::GetNeighborTilesAsArray(int32 Index, bool bBlockBorder) const
{
	if (!IsValidIndex(Index))
		return TArray<int32>();

	return GetNeighborTiles(Index, bBlockBorder).GetAllNodesAsArray(GridType, bSquareGridDiagonalAllowed);
}

TArray<int32> ADsGrid::GetNeighborTilesInRangeAsArray(int32 Index, int32 Range, bool bBlockBorder) const
{
	if (!IsValidIndex(Index))
		return TArray<int32>();

	TArray<int32> Out;

	if (Range > 0)
	{
		TArray<int32> Neighbors = GetNeighborTilesAsArray(Index, bBlockBorder);
		for (const auto& T : Neighbors)
			Out.AddUnique(T);

		if ((Range - 1) > 0)
		{
			for (int32 i = 0; i < (Range - 1); i++)
			{
				TArray<int32> TileNeighbors;
				for (const auto& Tile : Neighbors)
				{
					TArray<int32> Temp = GetNeighborTilesAsArray(Tile, bBlockBorder);
					for (const auto& T : Temp)
					{
						if (!Out.Contains(T) && T != Index)
						{
							TileNeighbors.AddUnique(T);
							Out.AddUnique(T);
						}
					}
				}

				Neighbors = TileNeighbors;
			}
		}
	}

	return Out;
}

int32 ADsGrid::GetNeighborTileIndex(int32 Index, ENeighborDirection Direction, ETileType InTileType) const
{
	if (!IsValidIndex(Index))
		return -1;

	FNeighbors Neighbors = GetNeighborTiles(Index);
	switch (Direction)
	{
	case ENeighborDirection::None:
		return -1;
	case ENeighborDirection::NORTH:
		if (InTileType == ETileType::Undefined)
			return Neighbors.NORTH;
		else if (GetNode(Neighbors.NORTH).NodeAttribute.TileType == InTileType)
			return Neighbors.NORTH;
		break;
	case ENeighborDirection::NORTH_EAST:
		if (InTileType == ETileType::Undefined)
			return Neighbors.NORTHEAST;
		else if (GetNode(Neighbors.NORTHEAST).NodeAttribute.TileType == InTileType)
			return Neighbors.NORTHEAST;
		break;
	case ENeighborDirection::EAST:
		if (GridType == EGridType::Hex)
			return -1;
		else if (InTileType == ETileType::Undefined)
			return Neighbors.EAST;
		else if (GetNode(Neighbors.EAST).NodeAttribute.TileType == InTileType)
			return Neighbors.EAST;
		break;
	case ENeighborDirection::SOUTH_EAST:
		if (InTileType == ETileType::Undefined)
			return Neighbors.SOUTHEAST;
		else if (GetNode(Neighbors.SOUTHEAST).NodeAttribute.TileType == InTileType)
			return Neighbors.SOUTHEAST;
		break;
	case ENeighborDirection::SOUTH:
		if (InTileType == ETileType::Undefined)
			return Neighbors.SOUTH;
		else if (GetNode(Neighbors.SOUTH).NodeAttribute.TileType == InTileType)
			return Neighbors.SOUTH;
		break;
	case ENeighborDirection::SOUTH_WEST:
		if (InTileType == ETileType::Undefined)
			return Neighbors.SOUTHWEST;
		else if (GetNode(Neighbors.SOUTHWEST).NodeAttribute.TileType == InTileType)
			return Neighbors.SOUTHWEST;
		break;
	case ENeighborDirection::WEST:
		if (GridType == EGridType::Hex)
			return -1;
		else if (InTileType == ETileType::Undefined)
			return Neighbors.WEST;
		else if (GetNode(Neighbors.WEST).NodeAttribute.TileType == InTileType)
			return Neighbors.WEST;
		break;
	case ENeighborDirection::NORTH_WEST:
		if (InTileType == ETileType::Undefined)
			return Neighbors.NORTHWEST;
		else if (GetNode(Neighbors.NORTHWEST).NodeAttribute.TileType == InTileType)
			return Neighbors.NORTHWEST;
		break;
	}

	return -1;
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
*   To Do: Change return val to struct. Return obstacle indices.
*/
FTileNeighborResult ADsGrid::GetNeighborIndexes(int32 Index, int32 EndIndex, FAStarPreferences Preferences) const
{
	SCOPE_CYCLE_COUNTER(STAT_GetNeighborIndexes);

	FTileNeighborResult Result;
	FNeighbors Neighbors = GetNeighborTiles(Index, Preferences.bBlockBorder);

	FNodeAttribute Access;

	if (Neighbors.EAST < (GridX * GridY) && Neighbors.EAST >= 0 && GridType == EGridType::Square)
	{
		Access = NodeBehavior(Index, Neighbors.EAST, EndIndex, Preferences, ENeighborDirection::EAST);
		if (Access.bAccess)
		{
			Result.Neighbors.Add(Neighbors.EAST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.EAST);
		}
	}
	if (Neighbors.WEST < (GridX * GridY) && Neighbors.WEST >= 0 && GridType == EGridType::Square)
	{
		Access = NodeBehavior(Index, Neighbors.WEST, EndIndex, Preferences, ENeighborDirection::WEST);
		if (Access.bAccess)
		{
			Result.Neighbors.Add(Neighbors.WEST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.WEST);
		}
	}
	if (Neighbors.SOUTH < (GridX * GridY) && Neighbors.SOUTH >= 0)
	{
		Access = NodeBehavior(Index, Neighbors.SOUTH, EndIndex, Preferences, ENeighborDirection::SOUTH);
		if (Access.bAccess)
		{
			Result.Neighbors.Add(Neighbors.SOUTH, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.SOUTH);
		}
	}
	if (Neighbors.NORTH < (GridX * GridY) && Neighbors.NORTH >= 0)
	{
		Access = NodeBehavior(Index, Neighbors.NORTH, EndIndex, Preferences, ENeighborDirection::NORTH);
		if (Access.bAccess)
		{
			Result.Neighbors.Add(Neighbors.NORTH, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.NORTH);
		}
	}

	if (Neighbors.SOUTHEAST < (GridX * GridY) && Neighbors.SOUTHEAST >= 0)
	{
		Access = NodeBehavior(Index, Neighbors.SOUTHEAST, EndIndex, Preferences, ENeighborDirection::SOUTH_EAST);
		if (Access.bAccess)
		{
			if (bSquareGridDiagonalAllowed && GridType == EGridType::Square)
			{
				Result.Neighbors.Add(Neighbors.SOUTHEAST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
			}
			else if (GridType == EGridType::Hex)
			{
				Result.Neighbors.Add(Neighbors.SOUTHEAST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
			}
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.SOUTHEAST);
		}
	}
	if (Neighbors.SOUTHWEST < (GridX * GridY) && Neighbors.SOUTHWEST >= 0)
	{
		Access = NodeBehavior(Index, Neighbors.SOUTHWEST, EndIndex, Preferences, ENeighborDirection::SOUTH_WEST);
		if (Access.bAccess)
		{
			if (bSquareGridDiagonalAllowed && GridType == EGridType::Square)
			{
				Result.Neighbors.Add(Neighbors.SOUTHWEST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
			}
			else if (GridType == EGridType::Hex)
			{
				Result.Neighbors.Add(Neighbors.SOUTHWEST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
			}
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.SOUTHWEST);
		}
	}
	if (Neighbors.NORTHWEST < (GridX * GridY) && Neighbors.NORTHWEST >= 0)
	{
		Access = NodeBehavior(Index, Neighbors.NORTHWEST, EndIndex, Preferences, ENeighborDirection::NORTH_WEST);
		if (Access.bAccess)
		{
			if (bSquareGridDiagonalAllowed && GridType == EGridType::Square)
			{
				Result.Neighbors.Add(Neighbors.NORTHWEST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
			}
			else if (GridType == EGridType::Hex)
			{
				Result.Neighbors.Add(Neighbors.NORTHWEST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
			}
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.NORTHWEST);
		}
	}
	if (Neighbors.NORTHEAST < (GridX * GridY) && Neighbors.NORTHEAST >= 0)
	{
		Access = NodeBehavior(Index, Neighbors.NORTHEAST, EndIndex, Preferences, ENeighborDirection::NORTH_EAST);
		if (Access.bAccess)
		{
			if (bSquareGridDiagonalAllowed && GridType == EGridType::Square)
			{
				Result.Neighbors.Add(Neighbors.NORTHEAST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
			}
			else if (GridType == EGridType::Hex)
			{
				Result.Neighbors.Add(Neighbors.NORTHEAST, FTileNeighborCost(Access.NodeCost, Access.NodeCostScale));
			}
		}
		else if (Preferences.bRecordObstacleIndexes)
		{
			Result.ObstacleIndexes.Add(Neighbors.NORTHEAST);
		}
	}

	return Result;
}

FNodeAttribute ADsGrid::NodeBehavior(int32 CurrentIndex, int32 NeighborIndex, int32 EndIndex, FAStarPreferences Preferences, ENeighborDirection Direction) const
{
	SCOPE_CYCLE_COUNTER(STAT_AccessNode);

	return Instances[CurrentIndex].NodeAttribute;
}

ENeighborDirection ADsGrid::GetNodeDirection(int32 CurrentIndex, int32 NextIndex) const
{
	//SCOPE_CYCLE_COUNTER(STAT_GetNodeDirection);

	ENeighborDirection Direction = ENeighborDirection::None;

	const FNeighbors Neighbors = GetNeighborTiles(CurrentIndex);

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

bool ADsGrid::SetTileType(int32 Index, ETileType NewTileType)
{
	if (!IsValidIndex(Index))
		return false;
	Instances[Index].NodeAttribute.TileType = NewTileType;
	OnTileAttributeChanged(Instances[Index]);
	return true;
}

bool ADsGrid::SetTileCost(int32 Index, float NewNodeCost)
{
	if (!IsValidIndex(Index))
		return false;
	Instances[Index].NodeAttribute.NodeCost = NewNodeCost;
	OnTileAttributeChanged(Instances[Index]);
	return true;
}

bool ADsGrid::SetTileAccess(int32 Index, bool bNewAccess)
{
	if (!IsValidIndex(Index))
		return false;
	Instances[Index].NodeAttribute.bAccess = bNewAccess;
	OnTileAttributeChanged(Instances[Index]);
	return true;
}

TArray<int32> ADsGrid::GetTileIndexWithSphere(FVector Center, float Radius) const
{
	return GetInstancesOverlappingSphere(Center, Radius);
}

TArray<int32> ADsGrid::GetTileIndicesWithBox(FVector Center, FVector Extent) const
{
	FBox Box;
	Box.BuildAABB(Center, Extent);
	return GetInstancesOverlappingBox(Box);
}

bool ADsGrid::IsValidIndex(int32 Index) const
{
	return Instances.Contains(Index);
}

FVector ADsGrid::GetTileLocation(int32 Index) const
{
	return GetTile(Index).Location;
}

FBox ADsGrid::GetTileBox(int32 Index) const
{
	if (!IsValidIndex(Index))
		return FBox();
	return TileBound.MoveTo(GetTile(Index).Location);
}

bool ADsGrid::SetTileAttribute(int32 Index, ETileType NewTileType, float NewNodeCost, bool bNewAccess)
{
	if (!IsValidIndex(Index))
		return false;

	Instances[Index].NodeAttribute.TileType = NewTileType;
	Instances[Index].NodeAttribute.NodeCost = NewNodeCost;
	Instances[Index].NodeAttribute.bAccess = bNewAccess;
	OnTileAttributeChanged(Instances[Index]);
	return true;
}

bool ADsGrid::SetTileAttributeByNode(int32 Index, FNodeAttribute NewProperty)
{
	if (!IsValidIndex(Index))
		return false;

	Instances[Index].NodeAttribute = NewProperty;
	OnTileAttributeChanged(Instances[Index]);
	return true;
}

bool ADsGrid::SetTilePropertyMap(TMap<int32, FNodeAttribute> NewNodeProperties)
{
	if (NewNodeProperties.Num() == 0)
		return false;
	for (const auto& Node : NewNodeProperties)
	{
		if (Instances.Contains(Node.Key))
		{
			Instances[Node.Key].NodeAttribute = Node.Value;
		}
	}
	OnTilePropertyMapSet();
	return true;
}

bool ADsGrid::SetTileZ(int32 Index, float Z)
{
	if (!IsValidIndex(Index))
		return false;
	Instances[Index].Location.Z = Z;

	return true;
}

int32 ADsGrid::GetIndexRow(int32 Index) const
{
	switch (TileOrder)
	{
	case EGridTileOrder::RowMajor:
		return Index / GridX;
	case EGridTileOrder::ColumnMajor:
		return Index % GridY;
	}
	return -1;
}

int32 ADsGrid::GetIndexColumn(int32 Index) const
{
	switch (TileOrder)
	{
	case EGridTileOrder::RowMajor:
		return Index % GridX;
	case EGridTileOrder::ColumnMajor:
		return Index / GridY;
	}
	return -1;
}

ETileType ADsGrid::GetTileType(int32 Index) const
{
	if (!IsValidIndex(Index))
		return ETileType::Undefined;
	return Instances[Index].NodeAttribute.TileType;
}

TArray<int32> ADsGrid::GetInstancesOverlappingBox(const FBox& Box) const
{
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

TArray<int32> ADsGrid::GetInstancesOverlappingSphere(const FVector& Center, const float Radius) const
{
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

int32 ADsGrid::GetTileIndex(FVector Point) const
{
	FBox CellBound = GetTileBound();
	CellBound.Min = TileBound.Min * FVector(TileScale, 1.0f);
	CellBound.Max = TileBound.Max * FVector(TileScale, 1.0f);
	for (const auto& Instance : Instances)
	{
		CellBound = CellBound.MoveTo(Instance.Value.Location);
		if (CellBound.IsInsideXY(Point))
		{
			return Instance.Key;
		}
	}
	return -1;
}

void ADsGrid::AddInstance(FVector Loc, FNodeAttribute NewNode)
{
	Instances.Add(Instances.Num(), FGridNode(Loc, NewNode));
}

void ADsGrid::CreateNewInstance(int32 Index, FNodeAttribute NewNode)
{
	const float boundX = TileBound.Max.X - TileBound.Min.X + TileOffset.X;
	const float boundY = TileBound.Max.Y - TileBound.Min.Y - TileOffset.Y;

	const FVector gridLoc = GetActorLocation();
	FVector NodeLoc = FVector::ZeroVector;

	switch (GridType)
	{
	case EGridType::Square:
	{
		switch (TileOrder)
		{
		case EGridTileOrder::RowMajor:
			NodeLoc.X = (Index % GridX) * boundX;
			NodeLoc.Y = (Index / GridX) * boundY;
			NodeLoc.Z = 0.0f;
			break;
		case EGridTileOrder::ColumnMajor:
			NodeLoc.X = (Index / GridY) * boundX;
			NodeLoc.Y = (Index % GridY) * boundY;
			NodeLoc.Z = 0.0f;
			break;
		default:
			break;
		}
	}
	break;
	case EGridType::Hex:
	{
		switch (TileOrder)
		{
		case EGridTileOrder::RowMajor:
			NodeLoc.X = (((Index / GridX) % 2) * (boundX / 2)) + ((Index % GridX) * boundX);
			NodeLoc.Y = (Index / GridX) * (boundY * 0.75);
			NodeLoc.Z = 0.0f;
			break;
		case EGridTileOrder::ColumnMajor:
			NodeLoc.X = (((Index % GridY) % 2) * (boundX / 2)) + ((Index / GridY) * boundX);
			NodeLoc.Y = (Index % GridY) * (boundY * 0.75);
			NodeLoc.Z = 0.0f;
			break;
		default:
			break;
		}
	}
	break;
	}

	AddInstance(gridLoc + ((NodeLoc * FVector(TileScale, 1.0f))), NewNode);
}

int32 ADsGrid::GetGridSize() const
{
	return Instances.Num();
}

void ADsGrid::ClearInstances()
{
	Instances.Empty();
}

FBox ADsGrid::GetTileBound() const
{
	return TileBound;
}
