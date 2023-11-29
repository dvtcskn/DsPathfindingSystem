/*
* DsPathfindingSystem
* Plugin code
* Copyright (c) 2023 Davut Coşkun
* All Rights Reserved.
*/

#include "DsGrid_PathFollowingComponent.h"

#include "VisualLogger/VisualLoggerTypes.h"
#include "VisualLogger/VisualLogger.h"

#include "UObject/Package.h"
#include "TimerManager.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/Controller.h"
#include "AISystem.h"
#include "BrainComponent.h"
#include "Engine/Canvas.h"
#include "VisualLogger/VisualLoggerTypes.h"
#include "VisualLogger/VisualLogger.h"
#include "Navigation/MetaNavMeshPath.h"
#include "AIConfig.h"

//#include "AI/Navigation/AbstractNavData.h"
//#include "AI/Navigation/NavLinkCustomInterface.h"
//#include "AI/Navigation/NavigationSystem.h"
//#include "AI/Navigation/RecastNavMesh.h"

#include "Navmesh/RecastNavMesh.h"
#include "AbstractNavData.h"
#include "NavLinkCustomInterface.h"
#include "NavigationSystem.h"

//#include "Navmesh/"

#include "DsAIController.h"

bool CheckPathIsSame(FNavPathSharedPtr InFirstPath, FNavPathSharedPtr InSecondPath)
{
	if (InFirstPath->GetPathPoints().Num() != InSecondPath->GetPathPoints().Num())
	{
		return false;
	}

	for (size_t i = 0; i < InFirstPath->GetPathPoints().Num(); i++)
	{
		auto FirstPath = InFirstPath->GetPathPoints()[i];
		auto SecondPath = InSecondPath->GetPathPoints()[i];

		if (FirstPath.Location != SecondPath.Location) {
			return false;
		}
	}
	return true;
}

UDsGrid_PathFollowingComponent::UDsGrid_PathFollowingComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{}

FAIRequestID UDsGrid_PathFollowingComponent::RequestMove(const FAIMoveRequest& RequestData, FNavPathSharedPtr InPath)
{
	auto LGetPathDescHelper = [&](FNavPathSharedPtr NavPath) -> FString
	{
		return !NavPath.IsValid() ? TEXT("missing") :
			!NavPath->IsValid() ? TEXT("invalid") :
			FString::Printf(TEXT("%s:%d"), NavPath->IsPartial() ? TEXT("partial") : TEXT("complete"), NavPath->GetPathPoints().Num());
	};

	UE_VLOG(GetOwner(), LogPathFollowing, Log, TEXT("RequestMove: Path(%s) %s"), *LGetPathDescHelper(InPath), *RequestData.ToString());
	LogPathHelper(GetOwner(), InPath, RequestData.GetGoalActor());

	if (ResourceLock.IsLocked())
	{
		UE_VLOG(GetOwner(), LogPathFollowing, Log, TEXT("Rejecting move request due to resource lock by %s"), *ResourceLock.GetLockPriorityName());
		return FAIRequestID::InvalidRequest;
	}

	const float UseAcceptanceRadius = (RequestData.GetAcceptanceRadius() == UPathFollowingComponent::DefaultAcceptanceRadius) ? MyDefaultAcceptanceRadius : RequestData.GetAcceptanceRadius();
	if (!InPath.IsValid() || UseAcceptanceRadius < 0.0f)
	{
		UE_VLOG(GetOwner(), LogPathFollowing, Log, TEXT("RequestMove: invalid request"));
		return FAIRequestID::InvalidRequest;
	}

	// try to grab movement component
	if (!UpdateMovementComponent())
	{
		UE_VLOG(GetOwner(), LogPathFollowing, Warning, TEXT("RequestMove: missing movement component"));
		return FAIRequestID::InvalidRequest;
	}

	FAIRequestID MoveId = GetCurrentRequestId();
	
	// Resume previous Path
	if (Status == EPathFollowingStatus::Paused && Path.IsValid() && CheckPathIsSame(InPath, Path) /*&& DestinationActor == RequestData.GetGoalActor()*/)
	{
		ResumeMove();
	}
	else
	{
		// --------------------------------------------- NEW PATH --------------------------------------------------------
		if (Status != EPathFollowingStatus::Idle)
		{
			// setting to false to make sure OnPathFinished won't result in stoping 
			bStopMovementOnFinish = false;
			OnPathFinished(FPathFollowingResult(EPathFollowingResult::Aborted, FPathFollowingResultFlags::NewRequest));
		}

		// force-setting this to true since this is the default state, and gets modified only 
		// when aborting movement.
		bStopMovementOnFinish = true;

		Reset();
		StoreRequestId();

		// store new Id, using CurrentRequestId directly at the end of function is not safe with chained moves
		MoveId = GetCurrentRequestId();

		// store new data
		Path = InPath;

		Path->AddObserver(FNavigationPath::FPathObserverDelegate::FDelegate::CreateUObject(this, &UPathFollowingComponent::OnPathEvent));
		if (MovementComp && MovementComp->GetOwner())
		{
			Path->SetSourceActor(*(MovementComp->GetOwner()));
		}

		OriginalMoveRequestGoalLocation = RequestData.GetGoalActor() ? RequestData.GetGoalActor()->GetActorLocation() : RequestData.GetGoalLocation();

		// update meta path data
		FMetaNavMeshPath* MetaNavPath = Path->CastPath<FMetaNavMeshPath>();
		if (MetaNavPath)
		{
			bIsUsingMetaPath = true;

			const FVector CurrentLocation = MovementComp ? MovementComp->GetActorFeetLocation() : FAISystem::InvalidLocation;
			MetaNavPath->Initialize(CurrentLocation);
		}

		PathTimeWhenPaused = 0.0f;
		OnPathUpdated();

		// make sure that OnPathUpdated didn't change current move request
		// otherwise there's no point in starting it
		if (GetCurrentRequestId() == MoveId)
		{
			AcceptanceRadius = UseAcceptanceRadius;
			bReachTestIncludesAgentRadius = RequestData.IsReachTestIncludingAgentRadius();
			bReachTestIncludesGoalRadius = RequestData.IsReachTestIncludingGoalRadius();
			GameData = RequestData.GetUserData();
			SetDestinationActor(RequestData.GetGoalActor());
			SetAcceptanceRadius(UseAcceptanceRadius);
			UpdateDecelerationData();

#if ENABLE_VISUAL_LOG
			const FVector CurrentLocation = MovementComp ? MovementComp->GetActorFeetLocation() : FVector::ZeroVector;
			const FVector DestLocation = InPath->GetDestinationLocation();
			const FVector ToDest = DestLocation - CurrentLocation;
			UE_VLOG(GetOwner(), LogPathFollowing, Log, TEXT("RequestMove: accepted, ID(%u) dist2D(%.0f) distZ(%.0f)"), MoveId, ToDest.Size2D(), FMath::Abs(ToDest.Z));
#endif // ENABLE_VISUAL_LOG

			if (!bIsUsingMetaPath && Path.IsValid() && Path->GetPathPoints().Num() > 0)
			{
				Status = EPathFollowingStatus::Moving;

				ADsAIController* AI = Cast<ADsAIController>(GetOwner());
				if (AI) {
					AI->OnBeginMovement();
				}

				// determine with path segment should be followed
				SetMoveSegment(0);
			}
			else
			{
				Status = EPathFollowingStatus::Waiting;
				GetWorld()->GetTimerManager().SetTimer(WaitingForPathTimer, this, &UDsGrid_PathFollowingComponent::OnWaitingPathTimeout, WaitingTimeout);
			}
		}
	}

	return MoveId;
}

void UDsGrid_PathFollowingComponent::OnWaitingPathTimeout()
{
	Super::OnWaitingPathTimeout();
}

void UDsGrid_PathFollowingComponent::AbortMove(const UObject& Instigator, FPathFollowingResultFlags::Type AbortFlags, FAIRequestID RequestID, EPathFollowingVelocityMode VelocityMode)
{
	// Empty because we don't want Abortmove immediately between two tiles
	//Super::AbortMove(Instigator, AbortFlags, RequestID, VelocityMode);
}

bool UDsGrid_PathFollowingComponent::DeterminePathStatus()
{
	if (Status == EPathFollowingStatus::Paused || Status == EPathFollowingStatus::Idle)
		return false;

	ADsAIController* AI = Cast<ADsAIController>(GetOwner());
	if (AI) {
		if (AI->IsPathNeedsToPause(MoveSegmentStartIndex)) {
			PauseMove(GetCurrentRequestId(), EPathFollowingVelocityMode::Keep);
		}

		if (Status == EPathFollowingStatus::Paused)
			return false;
	}

	return true;
}

void UDsGrid_PathFollowingComponent::OnPathFinished(const FPathFollowingResult & Result)
{
	Super::OnPathFinished(Result);

	ADsAIController* AI = Cast<ADsAIController>(GetOwner());
	if (AI) {
		AI->OnPathFinished();
	}

}

void UDsGrid_PathFollowingComponent::UpdatePathSegment()
{
	auto LLogBlockHelper = [&](AActor* LogOwner, UNavMovementComponent* MoveComp, float RadiusPct, float HeightPct, const FVector& SegmentStart, const FVector& SegmentEnd) -> void
	{
		#if ENABLE_VISUAL_LOG
		if (MoveComp && LogOwner)
		{
			const FVector AgentLocation = MoveComp->GetActorFeetLocation();
			const FVector ToTarget = (SegmentEnd - AgentLocation);
			const float SegmentDot = FVector::DotProduct(ToTarget.GetSafeNormal(), (SegmentEnd - SegmentStart).GetSafeNormal());
			UE_VLOG(LogOwner, LogPathFollowing, Verbose, TEXT("[agent to segment end] dot [segment dir]: %f"), SegmentDot);

			float AgentRadius = 0.0f;
			float AgentHalfHeight = 0.0f;
			AActor* MovingAgent = MoveComp->GetOwner();
			MovingAgent->GetSimpleCollisionCylinder(AgentRadius, AgentHalfHeight);

			const float Dist2D = ToTarget.Size2D();
			UE_VLOG(LogOwner, LogPathFollowing, Verbose, TEXT("dist 2d: %f (agent radius: %f [%f])"), Dist2D, AgentRadius, AgentRadius * (1 + RadiusPct));

			const float ZDiff = FMath::Abs(ToTarget.Z);
			UE_VLOG(LogOwner, LogPathFollowing, Verbose, TEXT("Z diff: %f (agent halfZ: %f [%f])"), ZDiff, AgentHalfHeight, AgentHalfHeight * (1 + HeightPct));
		}
		#endif // ENABLE_VISUAL_LOG
	};

	if ((Path.IsValid() == false) || (MovementComp == nullptr))
	{
		UE_CVLOG(Path.IsValid() == false, this, LogPathFollowing, Log, TEXT("Aborting move due to not having a valid path object"));
		OnPathFinished(FPathFollowingResult(EPathFollowingResult::Aborted, FPathFollowingResultFlags::InvalidPath));
		return;
	}

	FMetaNavMeshPath* MetaNavPath = bIsUsingMetaPath ? Path->CastPath<FMetaNavMeshPath>() : nullptr;

	/** it's possible that finishing this move request will result in another request
	*	which won't be easily detectable from this function. This simple local
	*	variable gives us this knowledge. */
	const FAIRequestID MoveRequestId = GetCurrentRequestId();

	// if agent has control over its movement, check finish conditions
	const FVector CurrentLocation = MovementComp->GetActorFeetLocation();
	const bool bCanUpdateState = HasMovementAuthority();
	if (bCanUpdateState && Status == EPathFollowingStatus::Moving)
	{
		const int32 LastSegmentEndIndex = Path->GetPathPoints().Num() - 1;
		const bool bFollowingLastSegment = (MoveSegmentStartIndex >= LastSegmentEndIndex);
		const bool bLastPathChunk = (MetaNavPath == nullptr || MetaNavPath->IsLastSection());

		if (bCollidedWithGoal)
		{
			// check if collided with goal actor
			OnSegmentFinished();
			OnPathFinished(FPathFollowingResult(EPathFollowingResult::Success, FPathFollowingResultFlags::None));
		}
		else if (MoveSegmentStartIndex > PreciseAcceptanceRadiusCheckStartNodeIndex && HasReachedDestination(CurrentLocation))
		{
			OnSegmentFinished();
			OnPathFinished(FPathFollowingResult(EPathFollowingResult::Success, FPathFollowingResultFlags::None));
		}
		else if (bFollowingLastSegment && bMoveToGoalOnLastSegment && bLastPathChunk)
		{
			// use goal actor for end of last path segment
			// UNLESS it's partial path (can't reach goal)
			if (DestinationActor.IsValid() && Path->IsPartial() == false)
			{
				const FVector AgentLocation = DestinationAgent ? DestinationAgent->GetNavAgentLocation() : DestinationActor->GetActorLocation();
				// note that the condition below requires GoalLocation to be in world space.
				const FVector GoalLocation = FQuatRotationTranslationMatrix(DestinationActor->GetActorQuat(), AgentLocation).TransformPosition(MoveOffset);

				CurrentDestination.Set(NULL, GoalLocation);

				UE_VLOG(this, LogPathFollowing, Log, TEXT("Moving directly to move goal rather than following last path segment"));
				UE_VLOG_LOCATION(this, LogPathFollowing, VeryVerbose, GoalLocation, 30, FColor::Green, TEXT("Last-segment-to-actor"));
				UE_VLOG_SEGMENT(this, LogPathFollowing, VeryVerbose, CurrentLocation, GoalLocation, FColor::Green, TEXT_EMPTY);
			}

			UpdateMoveFocus();
		}
		// check if current move segment is finished
		else if (HasReachedCurrentTarget(CurrentLocation))
		{
			OnSegmentFinished();

			if (DeterminePathStatus())
				SetNextMoveSegment();
		}
	}

	if (bCanUpdateState
		&& Status == EPathFollowingStatus::Moving
		// still the same move request
		&& MoveRequestId == GetCurrentRequestId())
	{
		// check waypoint switch condition in meta paths
		if (MetaNavPath)
		{
			MetaNavPath->ConditionalMoveToNextSection(CurrentLocation, EMetaPathUpdateReason::MoveTick);
		}

		// gather location samples to detect if moving agent is blocked
		const bool bHasNewSample = UpdateBlockDetection();
		if (bHasNewSample && IsBlocked())
		{
			if (Path->GetPathPoints().IsValidIndex(MoveSegmentEndIndex) && Path->GetPathPoints().IsValidIndex(MoveSegmentStartIndex))
			{
				LLogBlockHelper(GetOwner(), MovementComp, MinAgentRadiusPct, MinAgentHalfHeightPct,
					*Path->GetPathPointLocation(MoveSegmentStartIndex),
					*Path->GetPathPointLocation(MoveSegmentEndIndex));
			}
			else
			{
				if ((GetOwner() != NULL) && (MovementComp != NULL))
				{
					UE_VLOG(GetOwner(), LogPathFollowing, Verbose, TEXT("Path blocked, but move segment indices are not valid: start %d, end %d of %d"), MoveSegmentStartIndex, MoveSegmentEndIndex, Path->GetPathPoints().Num());
				}
			}
			OnPathFinished(FPathFollowingResult(EPathFollowingResult::Blocked, FPathFollowingResultFlags::None));
		}
	}
}

void UDsGrid_PathFollowingComponent::SetMoveSegment(int32 SegmentStartIndex)
{
	MoveSegmentStartIndex = SegmentStartIndex;

	int32 EndSegmentIndex = SegmentStartIndex + 1;
	const FNavigationPath* PathInstance = Path.Get();
	if (PathInstance != nullptr && PathInstance->GetPathPoints().IsValidIndex(SegmentStartIndex)/* && PathInstance->GetPathPoints().IsValidIndex(EndSegmentIndex)*/)
	{
		EndSegmentIndex = DetermineCurrentTargetPathPoint(SegmentStartIndex);

		MoveSegmentStartIndex = SegmentStartIndex;
		MoveSegmentEndIndex = EndSegmentIndex;
		const FNavPathPoint& PathPt0 = PathInstance->GetPathPoints()[MoveSegmentStartIndex];

		MoveSegmentStartRef = PathPt0.NodeRef;

		CurrentDestination = PathInstance->GetPathPointLocation(MoveSegmentStartIndex);

		const FVector SegmentStart = SegmentStartIndex == 0 ? MovementComp->GetActorFeetLocation() : *PathInstance->GetPathPointLocation(MoveSegmentStartIndex - 1);
		FVector SegmentEnd = SegmentStartIndex == 0 ? PathPt0.Location : *CurrentDestination;

		CurrentAcceptanceRadius = 0.0f;

		MoveSegmentDirection = (SegmentEnd - SegmentStart).GetSafeNormal();

		bWalkingNavLinkStart = FNavMeshNodeFlags(PathPt0.Flags).IsNavLink();

		// handle moving through custom nav links
		if (PathPt0.CustomLinkId)
		{
			UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
			INavLinkCustomInterface* CustomNavLink = NavSys->GetCustomLink(PathPt0.CustomLinkId);
			StartUsingCustomLink(CustomNavLink, SegmentEnd);
		}

		// update move focus in owning AI
		UpdateMoveFocus();
	}
}

void UDsGrid_PathFollowingComponent::OnSegmentFinished()
{
	UE_VLOG(GetOwner(), LogPathFollowing, Verbose, TEXT("OnSegmentFinished"));
	ADsAIController* AI = Cast<ADsAIController>(GetOwner());
	if (AI) {
		AI->OnSegmentFinished(MoveSegmentStartIndex);
	}
}

void UDsGrid_PathFollowingComponent::PauseMove(FAIRequestID RequestID, EPathFollowingVelocityMode VelocityMode)
{
	UE_VLOG(GetOwner(), LogPathFollowing, Log, TEXT("PauseMove: RequestID(%u)"), RequestID);
	if (Status == EPathFollowingStatus::Paused)
	{
		return;
	}

	if (RequestID.IsEquivalent(GetCurrentRequestId()))
	{
		if ((VelocityMode == EPathFollowingVelocityMode::Reset) && MovementComp && HasMovementAuthority())
		{
			MovementComp->StopMovementKeepPathing();
		}

		LocationWhenPaused = MovementComp ? MovementComp->GetActorFeetLocation() : FVector::ZeroVector;
		PathTimeWhenPaused = Path.IsValid() ? Path->GetTimeStamp() : 0.0f;
		Status = EPathFollowingStatus::Paused;

		UpdateMoveFocus();
	}
}

void UDsGrid_PathFollowingComponent::ResumeMove(FAIRequestID RequestID)
{
	if (RequestID.IsEquivalent(GetCurrentRequestId()) && RequestID.IsValid())
	{
		UE_VLOG(GetOwner(), LogPathFollowing, Log, TEXT("ResumeMove: RequestID(%u)"), RequestID);
		if (Path.IsValid() || Path->GetPathPoints().Num() > 0) {
			Status = EPathFollowingStatus::Moving;
			
			SetMoveSegment(MoveSegmentStartIndex + 1);
			UpdateMoveFocus();
		}
		else
		{
			OnPathFinished(FPathFollowingResult(EPathFollowingResult::OffPath, FPathFollowingResultFlags::None));
		}
	}
	else
	{
		UE_VLOG(GetOwner(), LogPathFollowing, Log, TEXT("ResumeMove: RequestID(%u) is neither \'AnyRequest\' not CurrentRequestId(%u). Ignoring."), RequestID, GetCurrentRequestId());
	}
}

