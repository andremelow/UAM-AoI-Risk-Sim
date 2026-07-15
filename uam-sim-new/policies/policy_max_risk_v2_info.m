function info = policy_max_risk_v2_info()
% POLICY_MAX_RISK_V2_INFO  Metadata for the drift-derived risk-aware Max-Weight
%  policy (switching/preemptive). Corrects policy_max_risk_sw's endogenous
%  weight, cross-contamination and ad-hoc anti-starvation.
info.mode = 'switching';
end
