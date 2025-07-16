#include <map>
#include <tuple>
#include "ctl_mission/J8_FSM.h"




    J8_FSM::J8_FSM() {
        // Initialize the transition table
        transitionTable = {
            {{Mode::Ready, Transition::ReadytoPath}, Mode::PathFollowing},
            {{Mode::PathFollowing, Transition::PathtoReady}, Mode::Ready},
            {{Mode::Ready, Transition::ReadytoTele}, Mode::Teleoperation},
            {{Mode::Teleoperation, Transition::TeletoReady}, Mode::Ready},
            {{Mode::GoingHome, Transition::HometoReady}, Mode::Ready},
            {{Mode::Ready, Transition::ReadytoHome}, Mode::GoingHome},
            {{Mode::EmergencyStop, Transition::EstoptoReady}, Mode::Ready},
            {{Mode::Ready, Transition::ReadytoRecordPath}, Mode::RecordPath},
            {{Mode::RecordPath, Transition::RecordPathtoReady}, Mode::Ready},
            {{Mode::Ready, Transition::AlltoEstop}, Mode::EmergencyStop}
            // ... other transitions
        };
        init_FSM();
    }

    void J8_FSM::init_FSM() {
        current_mode = Mode::Ready;
    }

    Mode J8_FSM::get_FSM_mode() const{
        return current_mode;
    }

Mode J8_FSM::Finite_Machine_State(Transition transition) {
    // Handle Emergency Stop transition separately
    if (transition == Transition::AlltoEstop) {
        current_mode = Mode::EmergencyStop;
    } else {
        // Handle other transitions using the transition table
        StateTransition key{current_mode, transition};
        auto it = transitionTable.find(key);

        if (it != transitionTable.end()) {
            // Transition found in the table, update the mode
            current_mode = it->second;
        } else {
            // Transition not found, you can handle this case appropriately
            // For example, log a warning or keep the current mode unchanged
        }
    }
    return current_mode;
}
std::vector<int> J8_FSM::get_possible_transitions() const {
    std::vector<int> possible_transitions(static_cast<int>(Mode::NumberOfModes), -1);

    for (const auto& entry : transitionTable) {
        // entry.first is a StateTransition struct
        if (entry.first.currentMode == current_mode) {
            int target_state_index = static_cast<int>(entry.second); // entry.second is the target Mode
            possible_transitions[target_state_index] = static_cast<int>(entry.first.transition); // entry.first.transition is the Transition
        }
    }

    // Always add the transition to EmergencyStop
    int emergency_stop_index = static_cast<int>(Mode::EmergencyStop);
    possible_transitions[emergency_stop_index] = static_cast<int>(Transition::AlltoEstop);

    return possible_transitions;
}