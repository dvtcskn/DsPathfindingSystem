/*
* DsPathfindingSystem
* Plugin code
* Copyright 2017-2018 Gereksiz
* All Rights Reserved.
*/

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "DsGrid_PathFollowingComponent.h"
#include "DsAIController.generated.h"

/**
 * 
 */
UCLASS(Blueprintable)
class DSPATHFINDINGSYSTEM_API ADsAIController : public AAIController
{
	GENERATED_BODY()

public:
	
	ADsAIController(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	// Move through the path
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation", Meta = (AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
		virtual EPathFollowingRequestResult::Type GridBasedMoveToLocation(const TArray<FVector>& Dests, float AcceptanceRadius = -1);

	// Resume Movement
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
		virtual void ResumeMovement();

	/*
	* We don't want to pause movement immediately. Because character can stop between 2 tiles.
	* We are setting 'bPauseMoveCalled' variable to true and 
	* When the character goes to the next 'Node', 'IsPathNeedsToPause' function is called and checks whether the path should be paused or not paused.
	*/
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
		virtual void PauseMovement();

	// When Movement start
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
		virtual void OnBeginMovement();

	// When Movement resume
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
		virtual void OnResumeMovement();

	// When Movement pause
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
		virtual void OnPauseMovement();

	// When Node finished
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
		virtual void OnSegmentFinished(int32 InSegmentIndex);

	// When Path finished
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
		virtual void OnPathFinished();

	// We don't want to pause movement immediately. Because character can stop between 2 tiles.
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
		virtual bool IsPathNeedsToPause(int32 InSegmentIndex);

	// Pause move called or not
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
		bool GetPauseMoveCalled() { return bPauseMoveCalled; };

protected:

	uint32 bPauseMoveCalled : 1;
};
