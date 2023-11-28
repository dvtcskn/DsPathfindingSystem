/*
* DsPathfindingSystem
* Plugin code
* Copyright 2017-2018 Gereksiz
* All Rights Reserved.
*/

#pragma once

//#include "CoreMinimal.h"
#include "Navigation/PathFollowingComponent.h"
#include "Navigation/MetaNavMeshPath.h"
#include "DsGrid_PathFollowingComponent.generated.h"

/**
 * 
 */
UCLASS()
class DSPATHFINDINGSYSTEM_API UDsGrid_PathFollowingComponent : public UPathFollowingComponent
{
	GENERATED_BODY()
	
public:

	UDsGrid_PathFollowingComponent(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	/** start movement along path
	* @return MoveId of requested move
	*/
	virtual FAIRequestID RequestMove(const FAIMoveRequest& RequestData, FNavPathSharedPtr InPath) override;

	/** called from timer if component spends too much time in Waiting state */
	virtual void OnWaitingPathTimeout() override;

	/*
	* Not working because we don't want Abortmove immediately between two tiles
	*/
	virtual void AbortMove(const UObject& Instigator, FPathFollowingResultFlags::Type AbortFlags, FAIRequestID RequestID = FAIRequestID::CurrentRequest, EPathFollowingVelocityMode VelocityMode = EPathFollowingVelocityMode::Reset) override;

	/** notify about finished movement */
	virtual void OnPathFinished(const FPathFollowingResult& Result);

	/** check state of path following, update move segment if needed */
	virtual void UpdatePathSegment() override;

	/** sets variables related to current move segment */
	virtual void SetMoveSegment(int32 SegmentStartIndex) override;

	/** pause path following
	*  @param RequestID - request to pause, FAIRequestID::CurrentRequest means pause current request, regardless of its ID */
	virtual void PauseMove(FAIRequestID RequestID = FAIRequestID::CurrentRequest, EPathFollowingVelocityMode VelocityMode = EPathFollowingVelocityMode::Reset) override;

	/** resume path following
	*  @param RequestID - request to resume, FAIRequestID::CurrentRequest means restor current request, regardless of its ID */
	virtual void ResumeMove(FAIRequestID RequestID = FAIRequestID::CurrentRequest) override;

	/** notify about finishing move along current path segment */
	virtual void OnSegmentFinished() override;

protected:

	virtual bool DeterminePathStatus();
};
