% define controller variant conditions
ctrl_m = enumeration('CtrlEnum');
ctrl_len = length(ctrl_m);
for i=1:ctrl_len
  cond_expr = sprintf('SimIn.ctrlType == CtrlEnum.%s', ctrl_m(i));
  switch ctrl_m(i)
    case CtrlEnum.TRIM
      GVS_CTRL_TYPE_TRIM = Simulink.Variant(cond_expr);
    case CtrlEnum.BASELINE
      GVS_CTRL_TYPE_BASELINE = Simulink.Variant(cond_expr);
    case CtrlEnum.BASELINE_L1
      GVS_CTRL_TYPE_BASELINE_L1 = Simulink.Variant(cond_expr);
    case CtrlEnum.BASELINE_AGI
      GVS_CTRL_TYPE_BASELINE_AGI = Simulink.Variant(cond_expr);
    case CtrlEnum.HOVER_BASELINE
      GVS_CTRL_TYPE_HOVER_BASELINE = Simulink.Variant(cond_expr);
    case CtrlEnum.CRUISE_BASELINE
      GVS_CTRL_TYPE_CRUISE_BASELINE = Simulink.Variant(cond_expr);
    case CtrlEnum.TRANSITION_BASELINE
      GVS_CTRL_TYPE_TRANSITION_BASELINE = Simulink.Variant(cond_expr);
    otherwise
  end
end

clear ctrl_m ctrl_len i cond_expr;