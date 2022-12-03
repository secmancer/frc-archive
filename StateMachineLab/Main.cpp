// Copyright (c) Adan Silva 2016. All Rights Reserved.

#include <iostream>

#include "RobotMock.hpp"
#include "Timer.hpp"

enum class State {
    Beginning, Tote1, Tote2, Tote3, End
};

int main() {
    constexpr double kGround = 0.0;
    constexpr double kHeightHalf = 7.2;
    constexpr double kHeight1 = 16.0;
    constexpr double kHeight2 = 28.76;

    RobotMock robot;
    State state = State::Beginning;  // Assign the initial state here
    Timer timer;

    while (1) {
        switch(state) {
            case State::Beginning:
                timer.Start();
                robot.SetClaw(false);
                robot.SetSetpoint(kGround);
                std::cout << "At Ground level" << std::endl;
                state = State::Tote1;
                break;
            case State::Tote1:
                if(robot.GetPressed()){
                    robot.SetClaw(true);
                    robot.SetSetpoint(kHeightHalf);
                    std::cout << "At half height" << std::endl;               
                }
                robot.SetClaw(false);
                robot.SetSetpoint(kGround);
                std::cout << "At Ground level" << std::endl;
                state = State::Tote2;
                break;
            case State::Tote2:
                if(robot.GetPressed()){
                    robot.SetClaw(true);
                    robot.SetSetpoint(kHeight1);
                    std::cout << "At height 1" << std::endl;
                }
                robot.SetClaw(false);
                robot.SetSetpoint(kGround);
                std::cout << "At Ground level" << std::endl;
                state = State::Tote3;
                break;
            case State::Tote3:
                if(robot.GetPressed()){
                    robot.SetClaw(true);
                    robot.SetSetpoint(kHeight2);
                    std::cout << "At height 2" << std::endl;                 
                }
                robot.SetClaw(false);
                robot.SetSetpoint(kGround);
                state = State::End;
                break;
            case State::End:
                std::cout << "Finished" << std::endl;
                break;
        }
    }
}
