/*
* DsPathfindingSystem
* Plugin code
* Copyright (c) 2023 Davut Coşkun
* All Rights Reserved.
*/

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "Delegates/DelegateCombinations.h"
#include "DsGrid_PathFollowingComponent.h"
#include "DsAIController.generated.h"

DECLARE_MULTICAST_DELEGATE_OneParam(FOnGridPathFinished, AActor*);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPathCompleted, int32, EndIndex, TEnumAsByte<EPathFollowingResult::Type>, Flag);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnSegmentFinished, int32, InSegmentIndex, int32, NextIndex);

/**
 * 
 */
UCLASS(Blueprintable)
class DSPATHFINDINGSYSTEM_API ADsAIController : public AAIController
{
	GENERATED_BODY()

public:	
	ADsAIController(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	//UPROPERTY(BlueprintAssignable)
	FOnGridPathFinished OnGridPathFinished;
	UPROPERTY(BlueprintAssignable)
	FOnPathCompleted OnPathCompleted;
	UPROPERTY(BlueprintAssignable)
	FOnSegmentFinished OnPathSegmentFinished;

	// Move through the path
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation", Meta = (AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
	virtual EPathFollowingRequestResult::Type GridBasedMoveToLocation(const TArray<FVector>& Dests, const TArray<int32>& Indexes, float AcceptanceRadius = -1);

	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
	void AbortMovement();

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
	virtual void OnPathFinished(TEnumAsByte<EPathFollowingResult::Type> Flag);

	// We don't want to pause movement immediately. Because character can stop between 2 tiles.
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
	virtual bool IsPathNeedsToPause(int32 InSegmentIndex);

	// Pause move called or not
	UFUNCTION(BlueprintCallable, Category = "DsPathfindingSystem|Navigation")
	bool GetPauseMoveCalled() { return bPauseMoveCalled; };

	UFUNCTION(BlueprintPure, Category = "DsPathfindingSystem|Navigation")
	TArray<int32> GetTileIndexes() const { return TileIndexes; }

protected:
	uint32 bPauseMoveCalled : 1;
	TArray<int32> TileIndexes;
};
