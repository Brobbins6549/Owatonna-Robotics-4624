#pragma once

#include "commonauto/AutoStep.h"
#include "navX/NavX.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

class level2 : public AutoStep {

    public:
        level2() : AutoStep("level2") {
        }

        void Init() {}
    
        bool Execute() {
            frc::SmartDashboard::PutNumber("Angle Value", NavX::GetInstance().getRoll());
            if (NavX::GetInstance().getRoll() < -8) {
                double speed = (.00033333333333333333 * NavX::GetInstance().getRoll() * NavX::GetInstance().getRoll()) + (.0016666666666 *  -NavX::GetInstance().getRoll());
                SwerveTrain::GetInstance().Drive(0, (1.35 * speed), 0, false, false, 1);
                return false;
            }
            else if (NavX::GetInstance().getRoll() > -8 && NavX::GetInstance().getRoll() < -3) {
                double speed = (.00033333333333333333 * NavX::GetInstance().getRoll() * NavX::GetInstance().getRoll()) + (.0016666666666 *  -NavX::GetInstance().getRoll());
                SwerveTrain::GetInstance().Drive(0, (1.15 * speed), 0, false, false, 1);
                return false;
            }
            else if (NavX::GetInstance().getRoll() > 8) {
                double speed = (.00033333333333333333 * NavX::GetInstance().getRoll() * NavX::GetInstance().getRoll()) + (.0016666666666 *  NavX::GetInstance().getRoll());
                SwerveTrain::GetInstance().Drive(0, (-1.25 * speed), 0, false, false, 1);
                return false;
            }
            else if (NavX::GetInstance().getRoll() < 8 && NavX::GetInstance().getRoll() > 3) {
                double speed = (.00033333333333333333 * NavX::GetInstance().getRoll() * NavX::GetInstance().getRoll()) + (.0016666666666 *  NavX::GetInstance().getRoll());
                SwerveTrain::GetInstance().Drive(0, (1.15 * speed), 0, false, false, 1);
                return false;
            }
            else if (NavX::GetInstance().getRoll() < 3 && NavX::GetInstance().getRoll() > -3) {
                SwerveTrain::GetInstance().Stop();
                return false;
            }
            else {
                SwerveTrain::GetInstance().Stop();
                return true;

            }
        }

    private:
};