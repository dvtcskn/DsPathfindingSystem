/*
* DsPathfindingSystem
* Plugin code
* Copyright (c) 2023 Davut Coşkun
* All Rights Reserved.
*/

#include "DsAIController.h"

ADsAIController::ADsAIController(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer.SetDefaultSubobjectClass<UDsGrid_PathFollowingComponent>(TEXT("PathFollowingComponent")))
	, bPauseMoveCalled(false)
{}

EPathFollowingRequestResult::Type ADsAIController::GridBasedMoveToLocation(const TArray<FVector>& Dests, const TArray<int32>& Indexes, float AcceptanceRadius)
{
	FPathFollowingRequestResult ResultData;
	ResultData.Code = EPathFollowingRequestResult::Failed;

	if (Dests.Num() <= 0)
		return ResultData;

	// abort active movement to keep only one request running
	if (GetPathFollowingComponent() && GetPathFollowingComponent()->GetStatus() != EPathFollowingStatus::Idle)
	{
		PauseMovement();
	}

	FAIMoveRequest MoveReq(Dests[Dests.Num() - 1]);
	MoveReq.SetUsePathfinding(true);
	MoveReq.SetAllowPartialPath(true);
	MoveReq.SetProjectGoalLocation(true);
	MoveReq.SetNavigationFilter(DefaultNavigationFilterClass);
	MoveReq.SetAcceptanceRadius(AcceptanceRadius);
	MoveReq.SetReachTestIncludesAgentRadius(true);
	MoveReq.SetCanStrafe(true);

	bPauseMoveCalled = false;

	{
		TSharedPtr<FNavigationPath, ESPMode::ThreadSafe> MetaPathPtr(new FNavigationPath(Dests, GetOwner()));
		FNavPathSharedPtr Path;
		Path = MetaPathPtr;
		MetaPathPtr = nullptr;

		const FAIRequestID RequestID = Path.IsValid() ? RequestMove(MoveReq, Path) : FAIRequestID::InvalidRequest;
		if (RequestID.IsValid())
		{
			TileIndexes = Indexes;

			bAllowStrafe = MoveReq.CanStrafe();
			ResultData.MoveId = RequestID;
			ResultData.Code = EPathFollowingRequestResult::RequestSuccessful;
		}
	}

	return ResultData;
}

void ADsAIController::AbortMovement()
{
	TileIndexes.Empty();
	StopMovement();

	/*if (GetPathFollowingComponent())
	{
		GetPathFollowingComponent()->AbortMove(*this, FPathFollowingResultFlags::Blocked | FPathFollowingResultFlags::ForcedScript);
	}*/
}

void ADsAIController::ResumeMovement()
{
	UDsGrid_PathFollowingComponent* PFC = Cast<UDsGrid_PathFollowingComponent>(GetPathFollowingComponent());
	if (PFC) {
		if (PFC->GetStatus() == EPathFollowingStatus::Idle || PFC->GetStatus() == EPathFollowingStatus::Paused) {
			PFC->ResumeMove();
			if (PFC->GetStatus() == EPathFollowingStatus::Moving) {
				bPauseMoveCalled = false;
				OnResumeMovement();
			}
		}
	}
}

void ADsAIController::PauseMovement()
{
	UDsGrid_PathFollowingComponent* PFC = Cast<UDsGrid_PathFollowingComponent>(GetPathFollowingComponent());
	if (PFC) {
		if (PFC->GetStatus() == EPathFollowingStatus::Moving) {
			bPauseMoveCalled = true;
			OnPauseMovement();
		}
	}
}

void ADsAIController::OnBeginMovement()
{

}

void ADsAIController::OnResumeMovement()
{

}

void ADsAIController::OnPauseMovement()
{

}

void ADsAIController::OnSegmentFinished(int32 InSegmentIndex)
{
	if (InSegmentIndex < 0 || InSegmentIndex >= TileIndexes.Num())
		return;

	const int32 Current = TileIndexes[InSegmentIndex];
	const int32 Next = InSegmentIndex + 1 < TileIndexes.Num() ? TileIndexes[InSegmentIndex + 1] : -1;
	OnPathSegmentFinished.Broadcast(Current, Next);
}

void ADsAIController::OnPathFinished(TEnumAsByte<EPathFollowingResult::Type> Flag)
{
	OnPathCompleted.Broadcast(TileIndexes.Num() > 0 ? TileIndexes.Last() : -1, Flag);
	TileIndexes.Empty();
	auto Temp = OnGridPathFinished;
	OnGridPathFinished.Clear();
	Temp.Broadcast(Cast<AActor>(GetPawn()));
	Temp.Clear();
}

bool ADsAIController::IsPathNeedsToPause(int32 InSegmentIndex)
{
	return bPauseMoveCalled;
}
