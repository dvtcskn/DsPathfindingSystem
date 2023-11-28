/*
* DsPathfindingSystem
* Plugin code
* Copyright 2017-2018 Gereksiz
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

AGrid::AGrid()
{
	gridLoc.X = -31500.0f;
	gridLoc.Y = -25000.0f;
	gridLoc.Z = 0.0f;

	cellScale.X = 2.0f;
	cellScale.Y = 2.0f;
	cellScale.Z = 1.0f;

	GridX = 128.0f;
	GridY = 128.0f;

	GridOffsetX = 0.0f;
	GridOffsetY = 0.0f;

	bVisible = false;

	scene = CreateDefaultSubobject <USceneComponent>(TEXT("USceneComponent"));
	scene->SetMobility(EComponentMobility::Static);
	RootComponent = scene;

	DummyComponentforCollision = CreateDefaultSubobject<UBoxComponent>(TEXT("Dummy Component for Collision"));
	DummyComponentforCollision->SetupAttachment(scene, "Dummy Component for Collision");
	DummyComponentforCollision->SetBoxExtent(FVector::ZeroVector, false);
	DummyComponentforCollision->SetMobility(EComponentMobility::Static);
	DummyComponentforCollision->BodyInstance.SetObjectType(ECC_WorldStatic);
	DummyComponentforCollision->BodyInstance.SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	DummyComponentforCollision->BodyInstance.SetResponseToChannel(ECC_WorldStatic, ECR_Block);
	DummyComponentforCollision->BodyInstance.SetResponseToChannel(ECC_WorldDynamic, ECR_Block);
	DummyComponentforCollision->BodyInstance.SetResponseToChannel(ECC_Pawn, ECR_Block);
	DummyComponentforCollision->BodyInstance.SetResponseToChannel(ECC_PhysicsBody, ECR_Block);
	DummyComponentforCollision->BodyInstance.SetResponseToChannel(ECC_Visibility, ECR_Ignore);

	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
}

void AGrid::OnConstruction(const FTransform & Transform)
{
	Super::OnConstruction(Transform);

	UnregisterAllComponents();

	FTransform NodeTransform;
	FVector NodeBoundMin;
	FVector NodeBoundMax;
	FVector NodeLocRef;

	float boundY = 0.0f;
	float boundX = 0.0f;
	int32 totalGridSize = 0;

	TotalGridSize = (GridX * GridY) - 1;

	TArray<FVector> NodeLocationArray;
	NodeLocationArray.Empty();
	
	UStaticMesh* Square = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), NULL, TEXT("StaticMesh'/DsPathfindingSystem/square.square'")));
	UStaticMesh* Hex = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), NULL, TEXT("StaticMesh'/DsPathfindingSystem/Hex.Hex'")));

	/*There is a bug if UHierarchicalInstancedStaticMeshComponent is created in constructor you can't use hISM->ClearInstances(); function
	engine crashes immediately Error is array out of bound */
	hISM = NewObject<UHierarchicalInstancedStaticMeshComponent>(this, TEXT("hISM"));

	if (GridMesh != nullptr && (GridMesh != Square && GridMesh != Hex)) {
		hISM->SetStaticMesh(GridMesh);
	}
	else {
		GridMesh = nullptr;
	}

	if(!GridMesh) 
	{
		switch (gridType)
		{
		case EGridType::E_Square:
		{
			if (!Square) {
				break;
			}
			GridMesh = Square;
			hISM->SetStaticMesh(GridMesh);
		}
		break;
		case EGridType::E_Hex:
		{
			if (!Hex) {
				break;
			}
			GridMesh = Hex;
			hISM->SetStaticMesh(GridMesh);
		}
		break;
		default:
			if (!GridMesh) {
				return;
			}
			break;
		}
	}

	if (!GridMesh) {
		return;
	}

	hISM->CastShadow = false;
	hISM->SetVisibility(bVisible, true);
	hISM->SetupAttachment(scene, "hISM");
	hISM->SetMobility(DummyComponentforCollision->Mobility);
	hISM->BodyInstance.SetObjectType(DummyComponentforCollision->GetCollisionObjectType());
	hISM->BodyInstance.SetCollisionEnabled(DummyComponentforCollision->GetCollisionEnabled());
	hISM->BodyInstance.SetResponseToChannels(DummyComponentforCollision->GetCollisionResponseToChannels());

	switch (gridType)
	{
	case EGridType::E_Square:

		if (!hISM->GetStaticMesh()) {
			return;
		}

		bHex = false;
		if (hISM->GetInstanceCount() >= 1) {
			hISM->ClearInstances();
		}
		hISM->GetLocalBounds(NodeBoundMin, NodeBoundMax);

		boundX = NodeBoundMax.X - NodeBoundMin.X + GridOffsetX;
		boundY = NodeBoundMax.Y - NodeBoundMin.Y - GridOffsetY;

		totalGridSize = (GridX*GridY) - 1;

		for (int32 ii = 0; ii <= totalGridSize; ++ii)
		{
			NodeLocRef.X = ((ii / GridY) * boundX);
			NodeLocRef.Y = ((ii % GridY) * boundY);
			NodeLocRef.Z = 0.0f;

			NodeLocationArray.AddUnique(NodeLocRef*cellScale);
		}

		for (auto& loc : NodeLocationArray)
		{
			NodeTransform.SetLocation(gridLoc - (-loc));
			NodeTransform.SetScale3D(cellScale);
			hISM->AddInstanceWorldSpace(NodeTransform);
		}
		break;

	case EGridType::E_Hex:

		if (!hISM->GetStaticMesh()) {
			return;
		}
		bHex = true;
		if (hISM->GetInstanceCount() >= 1) {
			hISM->ClearInstances();
		}
		hISM->GetLocalBounds(NodeBoundMin, NodeBoundMax);

		boundX = NodeBoundMax.X - NodeBoundMin.X + GridOffsetX;
		boundY = NodeBoundMax.Y - NodeBoundMin.Y - GridOffsetY;

		totalGridSize = (GridX*GridY) - 1;

		for (int32 ii = 0; ii <= totalGridSize; ++ii)
		{
			NodeLocRef.X = ((((ii / GridX) % 2) * (boundX / 2)) + ((ii % GridX) * boundX));
			NodeLocRef.Y = ((ii / GridX)*(boundY * 0.75));
			NodeLocRef.Z = 0.0f;

			NodeLocationArray.AddUnique(NodeLocRef*cellScale);
		}

		for (auto& loc : NodeLocationArray)
		{
			NodeTransform.SetLocation(gridLoc - (-loc));
			NodeTransform.SetScale3D(cellScale);
			hISM->AddInstanceWorldSpace(NodeTransform);
		}
		break;

	default:
		break;
	}

	RegisterAllComponents();
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

FVector AGrid::pureGetCellLocation(int32 index)
{
	FTransform Result;
	hISM->GetInstanceTransform(index, Result, true);
	return Result.GetLocation();
}

TArray<int32> AGrid::pureGetCellIndexWithSphere(FVector Center, float Radius)
{
	return hISM->GetInstancesOverlappingSphere(Center, Radius, true);
}

TArray<int32> AGrid::pureGetCellIndexWithBox(FVector Center, float Radius)
{
	FBox Box;
	FVector boxExtend/* = hISM->Bounds.BoxExtent * (Radius / 10)*/;
	boxExtend.X = hISM->Bounds.BoxExtent.X * (Radius / 10);
	boxExtend.Y = hISM->Bounds.BoxExtent.Y * (Radius / 10);
	boxExtend.Z = hISM->Bounds.BoxExtent.Z + hISM->Bounds.Origin.Z;
	Box.BuildAABB(Center, boxExtend);
	//DrawDebugBox(GetWorld(), Center, boxExtend, FColor(255, 0, 0), false, 60.0f/*,(uint8)100, 50.0f*/);
	return hISM->GetInstancesOverlappingBox(Box, true);
}

int32 AGrid::getNodeIndex(FVector targetVector)
{
	SCOPE_CYCLE_COUNTER(STAT_GetNodeIndex);

	UWorld* World	= GetWorld();
	int32 out		= NULL;
	float origin	= hISM->Bounds.Origin.Z;
	float boxExtend = hISM->Bounds.BoxExtent.Z;

	FVector sVector{ targetVector.X, targetVector.Y, ((boxExtend * 2) * -1.0f) + (origin + boxExtend) };
	FVector eVector{ targetVector.X, targetVector.Y, origin + boxExtend };

	TArray<FHitResult> traceHitResults;
	FCollisionQueryParams traceParams;
	traceParams.bTraceComplex = true;

	World->LineTraceMultiByObjectType(traceHitResults, sVector, eVector, hISM->GetCollisionObjectType(), traceParams);

	for (size_t i = 0; i < traceHitResults.Num(); i++)
	{
		if (traceHitResults[i].GetActor() != NULL) {
			if (traceHitResults[i].GetActor() == this) {
				if (traceHitResults[i].Item >= 0) {
					out = traceHitResults[i].Item;
					break;
				}
				else {
					out = traceHitResults[i].Item + FMath::Abs(32767.0f) + 32769.0f;
					break;
				}
			}
		}
	}
	if (out <= ((GridX * GridY) - 1) && out >= 0)
	{
		return out;
	}
	else {
		return -1;
	}
}

float AGrid::getActorZ(AActor * Actor, FVector Location, ECollisionChannel TraceType)
{
	float Z = -BIG_NUMBER;
	if (UWorld* world = GetWorld()) {
		TArray<FHitResult> traceHitResults;
		if (Actor) {
			FVector origin;
			FVector boxExtend;
			Actor->GetActorBounds(false, origin, boxExtend);
			FVector sVector{ Location.X,Location.Y, boxExtend.Z * 2 + Actor->GetActorLocation().Z };
			FVector eVector{ Location.X,Location.Y, Actor->GetActorLocation().Z - 1 };
			FCollisionQueryParams traceParams;
			traceParams.bTraceComplex = true;
			world->LineTraceMultiByObjectType(traceHitResults, sVector, eVector, UEngineTypes::ConvertToObjectType(TraceType), traceParams);
			for (size_t i = 0; i < traceHitResults.Num(); i++)
			{
				if (traceHitResults[i].GetActor() == Actor) {
					if (traceHitResults[i].Location.Z > Z) {
						Z = traceHitResults[i].Location.Z;
					}
				}
			}
		}
		return Z < 1 && Z > -1 ? 0 : Z;
	}
	return -BIG_NUMBER;
}

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
	auto Retrace = [&](const int32 start, const int32 end, const TMap<int32, FANode> &StructData) -> FAStarResult
	{
		int32 current = end;
		FAStarResult Result;

		while (current != start)
		{
			if (current <= -1) {
				Result.ResultState = EAStarResultState::SearchFail;
				return Result;
			}

			FVector currentVector = FIGetCellLocation(current);
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
	FCriticalSection		mutex;
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

		mutex.Lock();
		for (auto& Tile : GetNeighborIndexes(CurrentIndex, Preferences)) {

			auto& NextNode = GridGraph.FindOrAdd(Tile.Key);

			if (!NextNode.Closed)
			{
				float TraversalCost = (CurrentNode.TraversalCost + FIOctileDistance(FIGetCellLocation(CurrentIndex), FIGetCellLocation(Tile.Key))) + (Preferences.bFly || Preferences.bOverrideNodeCostToOne ? 1.0f : (Tile.Value * NodeCostScale));

				auto PredicateIndex = [&](const FANode fCostH) {return fCostH.OpenSetIndex == Tile.Key; };

				if (TraversalCost < NextNode.TraversalCost || !fCostHeap.ContainsByPredicate(PredicateIndex))
				{
					NextNode.NodeCost			= Preferences.bFly || Preferences.bOverrideNodeCostToOne ? 1.0f : Tile.Value;
					NextNode.TraversalCost		= TraversalCost;
					NextNode.parent				= CurrentIndex;
					NextNode.NodeCostCount		= CurrentNode.NodeCostCount + Preferences.bFly || Preferences.bOverrideNodeCostToOne ? 1.0f : Tile.Value;
					NextNode.parentCount		= CurrentNode.parentCount + 1;
					NextNode.OpenSetIndex		= Tile.Key;
					NextNode.HeuristicCost		= FIOctileDistance(FIGetCellLocation(Tile.Key), FIGetCellLocation(endIndex));	// NodePredicate
					NextNode.TotalCost			= NextNode.TraversalCost + NextNode.HeuristicCost;	// NodePredicate

					if (!fCostHeap.ContainsByPredicate(PredicateIndex))
					{
						fCostHeap.HeapPush(NextNode, Predicate);
					}
				}
			}
		}
		mutex.Unlock();
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
	} else {
		Result.ResultState = EAStarResultState::SearchFail;
		return Result;
	}

	for (auto& inx : openSet) {
		if (inx.Key != startIndex) {
			Result.PathResults.Add(FIGetCellLocation(inx.Key));
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

		if(currentIndex <= -1) {
			AResult.ResultState = EAStarResultState::SearchFail;
			return AResult;
		}

		FVector currentVector = FIGetCellLocation(currentIndex);
		AResult.PathResults.Insert(currentVector, 0);
		AResult.PathIndexes.Insert(currentIndex, 0);
		AResult.PathLength = AResult.PathLength + 1;
		if(StructData.PathCosts.Find(currentIndex)) {
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

UHierarchicalInstancedStaticMeshComponent* AGrid::getHISM()
{
	return hISM;
}

FNeighbors AGrid::calculateNeighborIndexes(int32 index)
{
	SCOPE_CYCLE_COUNTER(STAT_CalcNeighbor);

	FNeighbors Neighbors;
	int32 hex;

	switch (gridType)
	{
	case EGridType::E_Square:

		Neighbors.EAST		= GridY >= GridX ? (index + 1) % GridY == 0 ? -1 : index + 1 : (index + 1) % GridY == 0 ? -1 : index + 1;
		Neighbors.WEST		= GridY >= GridX ? index % GridY == 0 ? -1 : index - 1 : index % GridY == 0 ? -1 : index - 1;
		Neighbors.SOUTH		= GridY >= GridX ? index - GridY : index - GridY;
		Neighbors.NORTH		= GridY >= GridX ? index + GridY : index + GridY;
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
			Neighbors.NORTH		= (index + 1) % GridX == 0 ? -1 : (index + 1);
			Neighbors.SOUTH		= index % GridX == 0 ? -1 : (index - 1);
			Neighbors.EAST		= -1;
			Neighbors.WEST		= -1;
		}
		else {
			Neighbors.NORTHEAST = (GridX + index);
			Neighbors.NORTHWEST = (index - GridX);
			Neighbors.SOUTHWEST = /*(index - (GridX + 1))*/ index % GridX == 0 ? -1 : (index - (GridX + 1));
			Neighbors.SOUTHEAST = /*((GridX - (+1)) + index)*/ index % GridX == 0 ? -1 : ((GridX - (+1)) + index);
			Neighbors.NORTH		= (index + 1) % GridX == 0 ? -1 : (index + 1);
			Neighbors.SOUTH		= /*(index - 1)*/ index % GridX == 0 ? -1 : (index - 1);
			Neighbors.EAST		= -1;
			Neighbors.WEST		= -1;
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
	DiagonalBranch.Init(bHex || !Preferences.bDiagonal ? true : false, 8);

	TMap<int32, float> result;
	FNodeResult Access;

	if (Neighbors.EAST <= TotalGridSize && Neighbors.EAST >= 0 && !bHex) {
		Access = Preferences.bNodeBehavior_BlueprintOverride_PROTOTYPE_ONLY ? NodeBehaviorBlueprint(index, Neighbors.EAST, ENeighborDirection::E_EAST) : NodeBehavior(index, Neighbors.EAST, ENeighborDirection::E_EAST);
		if (Access.bAccess)
		{
			DiagonalBranch[0] = true;
			result.Add(Neighbors.EAST, Access.NodeCost);
		}
	}
	if (Neighbors.WEST <= TotalGridSize && Neighbors.WEST >= 0 && !bHex) {
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

FNodeResult AGrid::NodeBehavior(int32 CurrentIndex, int32 NeighborIndex, ENeighborDirection Direction)
{
	SCOPE_CYCLE_COUNTER(STAT_AccessNode);

	FNodeResult result;
	result.bAccess = true;
	result.NodeCost = 1.0f;

	return result;
}

FNodeResult AGrid::NodeBehaviorBlueprint_Implementation(int32 CurrentIndex, int32 NeighborIndex, ENeighborDirection Direction)
{
	SCOPE_CYCLE_COUNTER(STAT_AccessNodeBlueprint);

	FNodeResult result;
	result.bAccess = true;
	result.NodeCost = 1.0f;

	return result;
}
