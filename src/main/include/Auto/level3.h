#pragma once

#include "commonauto/AutoStep.h"
#include "navX/NavX.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

class level3 : public AutoStep {

    public:
        level3() : AutoStep("level3") {
        }

        void Init() {}
    
        bool Execute() {
            frc::SmartDashboard::PutNumber("Angle Value", NavX::GetInstance().getRoll());
            if (NavX::GetInstance().getRoll()){
                double RollAngleRadius = NavX::GetInstance().getRoll() * (M_PI / 180);
                SwerveTrain::GetInstance().Drive(0, (RollAngleRadius * -1), 0, false, false, 1);
                return false;
            }
        }
    private:
};