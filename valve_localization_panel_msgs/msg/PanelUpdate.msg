std_msgs/Header header

## Commands Defintion
uint8 Command

uint8 PLAN_GETREADY=0
uint8 PLAN_GRASP=1
uint8 PLAN_TURNING=2
uint8 PLAN_UNGRASP=3
uint8 PLAN_FINISH=4
uint8 PREVIEW=5
uint8 EXECUTE=6
uint8 NO_COMMAND=15

string State
string Info
float64 ValveRadius
float64 TurnAmount

## Valve Type Definition
string  ValveType
string ROUND="W"
string LEFT_LEVER="LL"
string RIGHT_LEVER="RL"

## End Effector Definition
string EndEffector
string GRIPPER="GRIPPER"
string PEG="PEG"

## Hand Usage Definition
string Hands
string PLANNER_BOTH="BH"
string PLANNER_LEFT="LH"
string PLANNER_RIGHT="RH"
string USER_BOTH="USER_BH"
string USER_LEFT="USER_LH"
string USER_RIGHT="USER_RH"

## Turn Direction Definition
string Direction
string CLOCKWISE="CW"
string COUNTER_CLOCKWISE="CCW"

## Planning in a Box
bool PlanInBox

## Complaince
bool LeftCompliance
bool RightCompliance

## Middle Grab
bool GrabMiddle
