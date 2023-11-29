/*
* DsPathfindingSystem
* Plugin code
* Copyright (c) 2023 Davut Coşkun
* All Rights Reserved.
*/

#include "DsPathfindingSystem.h"

#define LOCTEXT_NAMESPACE "FDsPathfindingSystemModule"

void FDsPathfindingSystemModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
}

void FDsPathfindingSystemModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FDsPathfindingSystemModule, DsPathfindingSystem)
