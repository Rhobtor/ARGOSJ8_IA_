// J8_FSM.h
#ifndef J8_FSM_H
#define J8_FSM_H

#include <map>
#include <tuple>
#include <vector>
// Define modes and transitions
enum class Mode {
    Ready = 0,
    PathFollowing,
    Teleoperation,
    GoingHome,
    EmergencyStop,
    RecordPath,
    NumberOfModes
    // ... other modes
};

enum class Transition {
    ReadytoPath = 0,
    PathtoReady,
    ReadytoTele,
    TeletoReady,
    ReadytoHome,
    HometoReady,
    ReadytoRecordPath,
    RecordPathtoReady,
    EstoptoReady,
    AlltoEstop
    // ... other transitions
};

// Define a structure for state transitions
struct StateTransition {
    Mode currentMode;
    Transition transition;

    bool operator<(const StateTransition& other) const {
        return std::tie(currentMode, transition) < std::tie(other.currentMode, other.transition);
    }
};

class J8_FSM {
    Mode current_mode;
    std::map<StateTransition, Mode> transitionTable;

public:
    J8_FSM();
    void init_FSM();
    Mode get_FSM_mode() const;
    Mode Finite_Machine_State(Transition transition);
    std::vector<int> get_possible_transitions() const;
};

#endif // J8_FSM_H

