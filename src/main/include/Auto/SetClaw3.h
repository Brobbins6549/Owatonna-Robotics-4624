#pragma once

#include "commonauto/AutoStep.h"

#include "Claw.h"

class SetClaw3 : public AutoStep {

    public:
        SetClaw3() : AutoStep("SetClaw2") {


        }

        void Init() {}

        bool Execute() {

            Claw::GetInstance().ClawTiltPositionDown(.9925);
            return true;
        }

    private:

};