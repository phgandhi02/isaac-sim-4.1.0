

// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2020-2024, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQueryOverlapBoxAnyDatabase.h>

#include "../plugins/SceneQueryShared.h"

using namespace omni::physx;
using namespace omni::physx::graph;

class OgnPhysXSceneQueryOverlapBoxAny
{
public:
    static bool compute(OgnPhysXSceneQueryOverlapBoxAnyDatabase& db)
    {
        // Read all inputs.
        carb::Float3 position;
        {
            const auto& input = db.inputs.position();
            if(!input.resolved())
            {
                db.logError("No position input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                position = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                position = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for position.");
                return false;
            }
        }

        carb::Float3 dimensions;
        {
            const auto& input = db.inputs.dimensions();
            if(!input.resolved())
            {
                db.logError("No dimensions input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                dimensions = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                dimensions = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for dimensions.");
                return false;
            }
        }

        // overlapBox uses half extent.
        dimensions.x *= 0.5f;
        dimensions.y *= 0.5f;
        dimensions.z *= 0.5f;

        GfRotation gf_rotation;
        {
            static const GfVec3d kAxes[] = { GfVec3d::XAxis(), GfVec3d::YAxis(), GfVec3d::ZAxis() };

            const auto& input = db.inputs.rotation();
            if(!input.resolved())
            {
                db.logError("No rotation input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                gf_rotation = GfRotation(kAxes[0], float3Input[0]) *
                              GfRotation(kAxes[1], float3Input[1]) *
                              GfRotation(kAxes[2], float3Input[2]);
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                gf_rotation = GfRotation(kAxes[0], double3Input[0]) *
                              GfRotation(kAxes[1], double3Input[1]) *
                              GfRotation(kAxes[2], double3Input[2]);
            }
            else
            {
                db.logError("Invalid data type input for rotation.");
                return false;
            }
        }
        carb::Float4 rotation = toFloat4(gf_rotation.GetQuaternion());

        db.outputs.overlap() = getPhysXSceneQuery()->overlapBox(dimensions, position, rotation, NULL, true);
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
