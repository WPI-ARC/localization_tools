localization_tools
==================

Frontend localization tools using interactive markers and backend tools using PCL and ICP to 'snap' markers against the environment.

Valve Planner Msgs
------------------

There two messages and two services that are defined for using the localiztion tools with the planner. They are:

Services:

1. `PlanTurning.srv` - No Changes in this version

2. `ExecuteTurning.srv` - No Changes in this version

Messges:

1. `PlannerRequest.msg` - *Added `string EndEffector` field with the options of "GRIPPER" or "PEG" to be sent*

2. `PlannerResponse.msg` - No Changes in this version


Valve_Localization
------------------

(Under Construction)

Current Menu Options with Cooresponding Planner Strings:

**There is now an option to use grippers or pegs. The "Hands" Menu changes for a round valve depending on the end_effector type.**

    Hands Menu:
    If the valve type is Round: (Sends ValveType = "W") (DEFAULT Option)
      If the end effector type is a gripper: (Sends EndEffector = "GRIPPER") (DEFAULT Option)
        Hands > Planner Assigns Both Hands (Sends Hands = "BH") (DEFAULT Option)
        Hands > Planner Assigns Left Hand Only (Sends Hands = "LH")
        Hands > Planner Assigns Right Hand Only (Sends Hands = "RH")
        Hands > User Defines Both Hands (Sends Hands = "USER_BH"`)
        Hands > User Defined Left Hand Only (Sends Hands = "USER_LH")
        Hands > User Defined Right Hand Only (Sends Hands = "USER_RH")
        Hands > SWITCH TO USING PEGS (Switches to EndEffector = PEG mode)
      If the end effector type is a peg: (Sends EndEffector = "PEG")
        Hands > User Defines Both Hands (Sends Hands = "USER_BH") (DEFAULT Option)
        Hands > User Defined Left Hand Only (Sends Hands = "USER_LH")
        Hands > User Defined Right Hand Only (Sends Hands = "USER_RH")
        Hands > SWITCH TO USING GRIPPERS (Switch to EndEffector = GRIPPER mode)
       
    If the valve type is Left Lever: (Sends ValveType = "LL")
      Hands > Planner Defined Right Hand Only (Sends Hands = "RH") (DEFAULT Option)
      Hands > Planner Defined Left Hand Only (Sends Hands = "LH")
      
    If the valve type is Right Lever: (Sends ValveType = "RL")
      Hands > Planner Defined Left Hand Only (Sends Hands = "LH") (DEFAULT Option)
      Hands > Planner Defined Right Hand Only (Sends Hands = "RH")
      
    
    Valve Type Menu:
    Valve Type > Round (Sends ValveType = "W")
    Valve Type > Left Lever (Sends ValveType = "LL")
    Valve Type > Right Level (Sends ValveType = "RL")
    
    
    Turn Direction Menu:
    Turn Direction > Turn Count-Clockwise (Sends Direction = "CCW")
    Turn Direction > Turn Clockwise (Sends Direction = "CW")


Upcoming Valve Localization Changes
-----------------------------------

1. Add in previous state that will record what the user has done and go back to that instead of the default options

2. Add the ability to change peg approach distance

3. Add "reset" option to grippers and pegs

4. Add visualization of grippers and pegs at the end of the arrows

5. Find and fix bugs
