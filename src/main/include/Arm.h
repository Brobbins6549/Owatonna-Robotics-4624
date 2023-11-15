#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "Consts.h"
#include <frc/AnalogPotentiometer.h>
#include <frc/PowerDistribution.h>

class Arm {

    public:
        static Arm& GetInstance() {
            static Arm* instance = new Arm(R_IntakeCANID1, R_PDH);
            return *instance;
        }

        void ArmSpeed(double speedToSet) {
            if(m_PDH->GetCurrent(10) <= 25){  // change int 14 to a different number when we know the correct port
                m_Intake1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);

            } 
            else{
                m_Intake1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.05);
            }
        }
        double Current(){
            return m_PDH->GetCurrent(10);
        }

    private:
        Arm(const int R_ArmActuatorCANID1, const int R_PDH) {
            m_Intake1 = new ctre::phoenix::motorcontrol::can::VictorSPX(R_IntakeCANID1);
            m_PDH = new frc::PowerDistribution{R_PDH, frc::PowerDistribution::ModuleType::kRev}; //talk to aiden about canid on the pdh and it having to be 1
        }
        
        ctre::phoenix::motorcontrol::can::VictorSPX *m_Intake1;
        frc::PowerDistribution *m_PDH;
};