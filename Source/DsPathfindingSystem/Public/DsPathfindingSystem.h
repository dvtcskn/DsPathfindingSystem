/*
* DsPathfindingSystem
* Plugin code
* Copyright 2017-2018 Gereksiz
* All Rights Reserved.
*/

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FDsPathfindingSystemModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};