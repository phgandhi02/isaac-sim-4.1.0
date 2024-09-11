

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

#include <OgnPhysXSceneQuerySweepSphereClosestDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "OgnPhysXSceneQuery.inl"

class OgnPhysXSceneQuerySweepSphereClosest : public OgnPhysXSceneQuery<OgnPhysXSceneQuerySweepSphereClosest, OgnPhysXSceneQuerySweepSphereClosestDatabase>
{
public:

    static void initialize(const GraphContextObj& context, const NodeObj& nodeObj)
    {
        SetConnectionCallbacks(context, nodeObj);
        
        // Since converting from paths to name tokens is expensive, we only generate these outputs if actually connected.
        // These outputs are also marked as deprecated and will be hidden, so they are only here for backwards compatibility.
        SetAttributeDeprecated(nodeObj, outputs::bodyPrimPath.m_token, "Body Prim Path output is deprecated. Please use the Body Prim output instead.");
        SetAttributeDeprecated(nodeObj, outputs::colliderPrimPath.m_token, "Collider Prim Path output is deprecated. Please use the Collider Prim output instead.");
        SetAttributeDeprecated(nodeObj, outputs::materialPath.m_token, "Material Path output is deprecated. Please use the Material Prim output instead.");
    }

    static bool compute(OgnPhysXSceneQuerySweepSphereClosestDatabase& db)
    {
        // Read all inputs.
        carb::Float3 origin;
        {
            const auto& input = db.inputs.origin();
            if(!input.resolved())
            {
                db.logError("No origin input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                origin = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                origin = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for origin.");
                return false;
            }
        }

        carb::Float3 direction;
        {
            const auto& input = db.inputs.direction();
            if(!input.resolved())
            {
                db.logError("No direction input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                direction = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                direction = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for direction.");
                return false;
            }
        }

        float radius;
        {
            const auto& input = db.inputs.radius();
            if(!input.resolved())
            {
                db.logError("No radius input supplied to node.");
                return false;
            }
            else if(auto floatInput = input.get<float>())
            {
                radius = *floatInput;
            }
            else if(auto doubleInput = input.get<double>())
            {
                radius = (float) *doubleInput;
            }
            else
            {
                db.logError("Invalid data type input for radius.");
                return false;
            }
        }

        float range;
        {
            const auto& input = db.inputs.sweepRange();
            if(!input.resolved())
            {
                range = PX_MAX_F32;
            }
            else if(auto floatInput = input.get<float>())
            {
                range = *floatInput;
            }
            else if(auto doubleInput = input.get<double>())
            {
                range = (float) *doubleInput;
            }
            else
            {
                db.logError("Invalid data type input for range.");
                return false;
            }
        }
        bool both_sides = db.inputs.bothSides();

        bool bHit = false;
        SweepHit hit;
        // SweepSphere reports an error when the range is 0, but as inputs may be generated dynamically (like by offsets) we want to be able to accept this.
        if(range != 0.0f)
        {
            bHit = getPhysXSceneQuery()->sweepSphereClosest(radius, origin, direction, range < 0.0f ? PX_MAX_F32 : range, hit, both_sides);
        }
        if(!bHit)
        {
            hit.collision = 0;
            hit.rigidBody = 0;
            hit.position = carb::Float3 {0.0f, 0.0f, 0.0f};
            hit.normal = carb::Float3 {1.0f, 0.0f, 0.0f};
            hit.distance = -1.0f;
            hit.faceIndex = 0;
            hit.material = 0;
        }

        auto& state = db.template sharedState<OgnPhysXSceneQuerySweepSphereClosest>();

        // Write the outputs.
        db.outputs.hit() = bHit;
        if(bHit)
        {
            db.outputs.colliderPrim().resize(1);
            db.outputs.colliderPrim()[0] = static_cast<omni::fabric::PathC>(hit.collision);
            if(GetIsDeprecatedAttributeConnected(db, outputs::colliderPrimPath.m_token)) db.outputs.colliderPrimPath() = asNameToken(hit.collision);
            db.outputs.bodyPrim().resize(1);
            db.outputs.bodyPrim()[0] = static_cast<omni::fabric::PathC>(hit.rigidBody);
            if(GetIsDeprecatedAttributeConnected(db, outputs::bodyPrimPath.m_token)) db.outputs.bodyPrimPath() = asNameToken(hit.rigidBody);
            db.outputs.position() = hit.position;
            db.outputs.normal() = hit.normal;
            db.outputs.distance() = hit.distance;
            db.outputs.faceIndex() = hit.faceIndex;
            db.outputs.materialPrim().resize(1);
            db.outputs.materialPrim()[0] = static_cast<omni::fabric::PathC>(hit.material);
            if(GetIsDeprecatedAttributeConnected(db, outputs::materialPath.m_token)) db.outputs.materialPath() = asNameToken(hit.material);
        }
        else
        {
            db.outputs.colliderPrim().resize(0);
            db.outputs.bodyPrim().resize(0);
            db.outputs.materialPrim().resize(0);
        }
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()

