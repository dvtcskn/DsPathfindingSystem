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

EPathFollowingRequestResult::Type ADsAIController::GridBasedMoveToLocation(const TArray<FVector>& Dests, float AcceptanceRadius)
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
	MoveReq.SetUsePathfinding(false);
	MoveReq.SetAllowPartialPath(false);
	MoveReq.SetProjectGoalLocation(false);
	MoveReq.SetNavigationFilter(DefaultNavigationFilterClass);
	MoveReq.SetAcceptanceRadius(AcceptanceRadius);
	MoveReq.SetReachTestIncludesAgentRadius(false);
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
			bAllowStrafe = MoveReq.CanStrafe();
			ResultData.MoveId = RequestID;
			ResultData.Code = EPathFollowingRequestResult::RequestSuccessful;
		}
	}

	return ResultData;
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
}

void ADsAIController::OnPathFinished()
{
}

bool ADsAIController::IsPathNeedsToPause(int32 InSegmentIndex)
{
	return bPauseMoveCalled;
}
