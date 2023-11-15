#pragma once

#include "commonauto/AutoStep.h"
#include "navX/NavX.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

class level : public AutoStep {

    public:
        level() : AutoStep("level") {
        }

        void Init() {}
    
        bool Execute() {
            frc::SmartDashboard::PutNumber("Angle Value", NavX::GetInstance().getPitch());
            if (NavX::GetInstance().getPitch() < -8) {
                double speed = (.00033333333333333333 * NavX::GetInstance().getPitch() * NavX::GetInstance().getPitch()) + (.0016666666666 *  -NavX::GetInstance().getPitch());
                SwerveTrain::GetInstance().Drive(0, (1.35 * speed), 0, false, false, 1);
                return false;
            }
            else if (NavX::GetInstance().getPitch() > -8 && NavX::GetInstance().getPitch() < -3) {
                double speed = (.00033333333333333333 * NavX::GetInstance().getPitch() * NavX::GetInstance().getPitch()) + (.0016666666666 *  -NavX::GetInstance().getPitch());
                SwerveTrain::GetInstance().Drive(0, (1.15 * speed), 0, false, false, 1);
                return false;
            }
            else if (NavX::GetInstance().getPitch() > 8) {
                double speed = (.00033333333333333333 * NavX::GetInstance().getPitch() * NavX::GetInstance().getPitch()) + (.0016666666666 *  NavX::GetInstance().getPitch());
                SwerveTrain::GetInstance().Drive(0, (-1.25 * speed), 0, false, false, 1);
                return false;
            }
            else if (NavX::GetInstance().getPitch() < 8 && NavX::GetInstance().getPitch() > 3) {
                double speed = (.00033333333333333333 * NavX::GetInstance().getPitch() * NavX::GetInstance().getPitch()) + (.0016666666666 *  NavX::GetInstance().getPitch());
                SwerveTrain::GetInstance().Drive(0, (1.15 * speed), 0, false, false, 1);
                return false;
            }
            else if (NavX::GetInstance().getPitch() < 3 && NavX::GetInstance().getPitch() > -3) {
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