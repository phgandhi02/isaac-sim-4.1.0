// Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto. Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <omni/isaac/utils/BaseResetNode.h>

#include <OgnDifferentialControllerDatabase.h>
#include <cmath>

namespace omni
{
namespace isaac
{
namespace wheeled_robot
{

class OgnDifferentialController : public BaseResetNode
{

public:
    static bool compute(OgnDifferentialControllerDatabase& db)
    {
        auto& state = db.perInstanceState<OgnDifferentialController>();
        state.nodeObj = db.abi_node();

        if (!state.mInitialized)
        {
            if (db.inputs.wheelRadius() <= 0 || db.inputs.wheelDistance() <= 0)
            {
                db.logWarning("invalid wheel radius and distance");
                return false;
            }
            else
            {
                state.mWheelRadius = db.inputs.wheelRadius();
                state.mWheelDistance = db.inputs.wheelDistance();
            }

            if (std::fabs(db.inputs.maxLinearSpeed()) > 0)
            {
                state.mMaxLinearSpeed = std::fabs(db.inputs.maxLinearSpeed());
            }

            if (std::fabs(db.inputs.maxAcceleration()) > 0)
            {
                state.mMaxAcceleration = std::fabs(db.inputs.maxAcceleration());
            }

            if (std::fabs(db.inputs.maxDeceleration()) > 0)
            {
                state.mMaxDeceleration = std::fabs(db.inputs.maxDeceleration());
            }

            if (std::fabs(db.inputs.maxAngularAcceleration()) > 0)
            {
                state.mMaxAngularAcceleration = std::fabs(db.inputs.maxAngularAcceleration());
            }

            if (std::fabs(db.inputs.maxAngularSpeed()) > 0)
            {
                state.mMaxAngularSpeed = std::fabs(db.inputs.maxAngularSpeed());
            }

            if (std::fabs(db.inputs.maxWheelSpeed()) > 0)
            {
                state.mMaxWheelSpeed = std::fabs(db.inputs.maxWheelSpeed());
            }

            state.mPreviousAngularSpeed = 0;
            state.mPreviousAngularSpeed = 0;
            state.mInitialized = true;
        }
        double dt = db.inputs.dt();

        // if dt is invalid, but acceleration checks are required, skip compute
        if (dt <= 0.0 &&
            (state.mMaxAcceleration != 0.0 || state.mMaxDeceleration != 0.0 || state.mMaxAngularAcceleration != 0.0))
        {
            db.logWarning("invalid dt %f, cannot check for acceleration limits, skipping current step", dt);
            return false;
        }

        // if the command velocity is higher than max velocity, clip the command velocity to max velocity
        double linearVelocity =
            std::max(-state.mMaxLinearSpeed, std::min(state.mMaxLinearSpeed, db.inputs.linearVelocity()));
        double angularVelocity =
            std::max(-state.mMaxAngularSpeed, std::min(state.mMaxAngularSpeed, db.inputs.angularVelocity()));

        if (state.mMaxAcceleration != 0.0 && state.mMaxDeceleration != 0.0)
        {
            // if the magnitude is increasing, and the sign is the same, the robot is accelerating
            if (std::fabs(linearVelocity) > std::fabs(state.mPreviousLinearSpeed) &&
                linearVelocity * state.mPreviousLinearSpeed >= 0)
            {
                linearVelocity =
                    std::max(state.mPreviousLinearSpeed - state.mMaxAcceleration * dt,
                             std::min(linearVelocity, state.mPreviousLinearSpeed + state.mMaxAcceleration * dt));
            }
            // if the magnitude is decreasing, or the sign is not the same, the robot is braking (decelerating)
            else
            {
                linearVelocity =
                    std::max(state.mPreviousLinearSpeed - state.mMaxDeceleration * dt,
                             std::min(linearVelocity, state.mPreviousLinearSpeed + state.mMaxDeceleration * dt));
            }
        }

        if (state.mMaxAngularAcceleration != 0.0)
        {
            angularVelocity =
                std::max(state.mPreviousAngularSpeed - state.mMaxAngularAcceleration * dt,
                         std::min(angularVelocity, state.mPreviousAngularSpeed + state.mMaxAngularAcceleration * dt));
        }

        state.mPreviousAngularSpeed = angularVelocity;
        state.mPreviousLinearSpeed = linearVelocity;
        // calculate wheel speed
        auto& jointVelocities = db.outputs.velocityCommand();
        jointVelocities.resize(2);
        jointVelocities[0] = ((2 * linearVelocity) - (angularVelocity * state.mWheelDistance)) / (2 * state.mWheelRadius);
        jointVelocities[1] = ((2 * linearVelocity) + (angularVelocity * state.mWheelDistance)) / (2 * state.mWheelRadius);

        jointVelocities[0] = std::max(-state.mMaxWheelSpeed, std::min(state.mMaxWheelSpeed, jointVelocities[0]));
        jointVelocities[1] = std::max(-state.mMaxWheelSpeed, std::min(state.mMaxWheelSpeed, jointVelocities[1]));

        return true;
    }

    virtual void reset()
    {

        if (!nodeObj.iNode)
        {
            return;
        }
        GraphObj graphObj{ nodeObj.iNode->getGraph(nodeObj) };
        GraphContextObj context{ graphObj.iGraph->getDefaultGraphContext(graphObj) };

        // set the node's input and output
        AttributeObj linearAttr = nodeObj.iNode->getAttribute(nodeObj, "inputs:linearVelocity");
        auto linearHandle = linearAttr.iAttribute->getAttributeDataHandle(linearAttr, kAccordingToContextIndex);
        double* linearVelocity = getDataW<double>(context, linearHandle);
        if (linearVelocity)
        {
            *linearVelocity = 0;
        }

        // set the node's input and output
        AttributeObj angularAttr = nodeObj.iNode->getAttribute(nodeObj, "inputs:angularVelocity");
        auto angularHandle = angularAttr.iAttribute->getAttributeDataHandle(angularAttr, kAccordingToContextIndex);
        double* angularVelocity = getDataW<double>(context, angularHandle);
        if (angularVelocity)
        {
            *angularVelocity = 0;
        }

        mPreviousLinearSpeed = 0.0;
        mPreviousAngularSpeed = 0.0;
        mInitialized = false;

        // set the node's input and output
        // TODO: Disabled because this causes a crash on reset when next compute is called
        // AttributeObj velocityAttr = nodeObj.iNode->getAttribute(nodeObj, "outputs:velocityCommand");
        // auto velocityHandle = velocityAttr.iAttribute->getAttributeDataHandle(velocityAttr,
        // kAccordingToContextIndex); double* velocityCommand = getDataW<double>(context, velocityHandle);
        // velocityCommand[0] = 0.0;
        // velocityCommand[1] = 0.0;
    }

private:
    bool mInitialized = false;
    double mMaxAngularSpeed = 1.0e7;
    double mMaxWheelSpeed = 1.0e7;
    double mMaxLinearSpeed = 1.0e7;
    double mMaxAcceleration = 0.0;
    double mMaxDeceleration = 0.0;
    double mMaxAngularAcceleration = 0.0;
    double mPreviousLinearSpeed = 0.0;
    double mPreviousAngularSpeed = 0.0;
    double mWheelDistance = 0;
    double mWheelRadius = 0;
    NodeObj nodeObj;
};

REGISTER_OGN_NODE()
}
}
}
