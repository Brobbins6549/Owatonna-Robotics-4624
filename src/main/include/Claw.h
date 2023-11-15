#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "Consts.h"
#include <frc/Solenoid.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/PowerDistribution.h>

enum ClawState {
    IDLE,
    MOVING_UP,
    MOVING_DOWN
};

class Claw {
    public:
        static Claw& GetInstance() {
            static Claw* instance = new Claw(R_ClawActuatorCANID1, R_ClawActuatorCANID2, R_ClawPotentiometer);
            return *instance;
        }

        void ClawTiltSpeed(double speedToSet) {
            m_ClawActuator1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
            m_ClawActuator2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
            frc::SmartDashboard::PutNumber("Potentimeter", m_ClawPotentiometer->Get());
        }   

        void ClawTiltPositionUp(double Distancetoset){
            m_TargetPotentiometerValue = Distancetoset;
            m_CurrentState = MOVING_UP;
        }

        void ClawTiltPositionDown(double Distancetoset){
            m_TargetPotentiometerValue = Distancetoset;
            m_CurrentState = MOVING_DOWN;
        }
        void Idle(){
            m_CurrentState = IDLE;
        }

void Update() {
    switch (m_CurrentState) {
        case IDLE:
            frc::SmartDashboard::PutString("Update", "Idle");
            m_ClawActuator1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            m_ClawActuator2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            break;

        case MOVING_UP:
            frc::SmartDashboard::PutString("Update", "Up");
            if (m_ClawPotentiometer->Get() >= .695) {
                Claw::GetInstance().Idle();
                Claw::GetInstance().Update();
            }
            else {
                m_ClawActuator1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
                m_ClawActuator2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
            }
            break;

        case MOVING_DOWN:
            frc::SmartDashboard::PutString("Update", "Down");
            if (m_ClawPotentiometer->Get() <= .505) {
                Claw::GetInstance().Idle();
                Claw::GetInstance().Update();
            }
            else {
                m_ClawActuator1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
                m_ClawActuator2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
            }
            break;
    }
    frc::SmartDashboard::PutNumber("Potentimeter", m_ClawPotentiometer->Get());
}


    private:
        Claw(const int R_ClawActuatorCANID1, const int R_ClawActuatorCANID2, const int R_ClawPotentiometer) {
            m_ClawActuator1 = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClawActuatorCANID1);
            m_ClawActuator2 = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClawActuatorCANID2);
            m_ClawPotentiometer = new frc::AnalogPotentiometer(R_ClawPotentiometer, 1.0, 0.0 );
        }
        
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClawActuator1;
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClawActuator2;
        frc::AnalogPotentiometer *m_ClawPotentiometer;
        double m_TargetPotentiometerValue;
        ClawState m_CurrentState;
};    
 