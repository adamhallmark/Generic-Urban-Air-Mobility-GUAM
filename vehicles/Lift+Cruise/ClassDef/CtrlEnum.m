classdef CtrlEnum < Simulink.IntEnumType
  enumeration
    TRIM(1)
    BASELINE(2)
    BASELINE_L1(3)
    BASELINE_AGI(4)
    HOVER_BASELINE(5)
    CRUISE_BASELINE(6)
    TRANSITION_BASELINE(7)
  end
end
