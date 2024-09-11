

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

#include <OgnPhysXSceneQueryOverlapSphereAllDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "OgnPhysXSceneQuery.inl"

class OgnPhysXSceneQueryOverlapSphereAll : public OgnPhysXSceneQuery<OgnPhysXSceneQueryOverlapSphereAll, OgnPhysXSceneQueryOverlapSphereAllDatabase>
{
public:

    static void initialize(const GraphContextObj& context, const NodeObj& nodeObj)
    {
        SetConnectionCallbacks(context, nodeObj);
        
        // Since converting from paths to name tokens is expensive, we only generate these outputs if actually connected.
        // These outputs are also marked as deprecated and will be hidden, so they are only here for backwards compatibility.
        SetAttributeDeprecated(nodeObj, outputs::bodyPrimPaths.m_token, "Body Prim Paths output is deprecated. Please use the Body Prims output instead.");
        SetAttributeDeprecated(nodeObj, outputs::colliderPrimPaths.m_token, "Collider Prim Paths output is deprecated. Please use the Collider Prims output instead.");
    }

    static bool compute(OgnPhysXSceneQueryOverlapSphereAllDatabase& db)
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

        std::vector<OverlapHit> gatherList;
        auto gather = [&gatherList](const OverlapHit& hit) -> bool { gatherList.push_back(hit); return true;} ;

        getPhysXSceneQuery()->overlapSphere(radius, position, gather, false);

        auto& state = db.template sharedState<OgnPhysXSceneQueryOverlapSphereAll>();

        // Write the outputs.
        int n = 0;
        db.outputs.colliderPrims().resize(gatherList.size());
        db.outputs.bodyPrims().resize(gatherList.size());
        bool bOutputCollidersAsTokens = GetIsDeprecatedAttributeConnected(db, outputs::colliderPrimPaths.m_token);
        bool bOutputBodiesAsTokens = GetIsDeprecatedAttributeConnected(db, outputs::bodyPrimPaths.m_token);
        db.outputs.colliderPrimPaths().resize(bOutputCollidersAsTokens ? gatherList.size() : 0);
        db.outputs.bodyPrimPaths().resize(bOutputBodiesAsTokens ? gatherList.size() : 0);
        for (const OverlapHit& hit : gatherList)
        {
            db.outputs.colliderPrims()[n] = static_cast<omni::fabric::PathC>(hit.collision);
            db.outputs.bodyPrims()[n] = static_cast<omni::fabric::PathC>(hit.rigidBody);
            if(bOutputCollidersAsTokens) db.outputs.colliderPrimPaths()[n] = asNameToken(hit.collision);
            if(bOutputBodiesAsTokens) db.outputs.bodyPrimPaths()[n] = asNameToken(hit.rigidBody);
            n++;
        }
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
