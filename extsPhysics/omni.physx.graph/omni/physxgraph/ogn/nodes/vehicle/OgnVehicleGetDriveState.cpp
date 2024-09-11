// Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#include "UsdPCH.h"

#include <carb/Framework.h>

#include <omni/physx/IPhysxVehicle.h>
#include <omni/physx/IPhysx.h>

// include auto-generated header
#include <OgnVehicleGetDriveStateDatabase.h>


class OgnVehicleGetDriveState
{
private:

public:
    static bool compute(OgnVehicleGetDriveStateDatabase& db)
    {
        omni::physx::IPhysx* physXInterface = carb::getCachedInterface<omni::physx::IPhysx>();
        if (!physXInterface)
        {
            db.logError("Could not retrieve omni::physx::IPhysxVehicle interface");
            return false;
        }

        const NameToken pathToken = db.inputs.vehiclePath();
        const char* vehiclePath = db.tokenToString(pathToken);
        if (!vehiclePath)
        {
            db.logError("The vehicle prim path is not defined");
            return false;
        }

        const omni::physx::usdparser::ObjectId vehicleControllerId = physXInterface->getObjectId(pxr::SdfPath(vehiclePath), omni::physx::ePTVehicleController);
        if (omni::physx::usdparser::kInvalidObjectId == vehicleControllerId)
        {
            db.logError("Accessing object id for prim at \"%s\" failed", vehiclePath);
            return false;
        }

        omni::physx::VehicleDriveState driveState;
        if (!physXInterface->getVehicleDriveState(vehicleControllerId, driveState))
        {
            db.logError("Accessing drive state for prim at \"%s\" failed", vehiclePath);
            return false;
        }

        float& accelerator = db.outputs.accelerator();
        float& brake0 = db.outputs.brake0();
        float& brake1 = db.outputs.brake1();
        float& steer = db.outputs.steer();
        int& currentGear = db.outputs.currentGear();
        int& targetGear = db.outputs.targetGear();
        bool& automaticTransmission = db.outputs.automaticTransmission();
        float& engineRotationSpeed = db.outputs.engineRotationSpeed();

        accelerator = driveState.accelerator;
        brake0 = driveState.brake0;
        brake1 = driveState.brake1;
        steer = driveState.steer;
        currentGear = driveState.currentGear;
        targetGear = driveState.targetGear;
        automaticTransmission = driveState.automaticTransmission;
        engineRotationSpeed = driveState.engineRotationSpeed;

        return true;
    }
};

REGISTER_OGN_NODE()
