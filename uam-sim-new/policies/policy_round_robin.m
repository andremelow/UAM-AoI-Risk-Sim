function [txSlots, rrPointer] = policy_round_robin(activeDrones, K, rrPointer)
% POLICY_ROUND_ROBIN  Baseline equal-time allocation.

n = numel(activeDrones);
txSlots = zeros(1, K);
ptr = mod(rrPointer - 1, n) + 1;
for s = 1:K
    txSlots(s) = activeDrones(ptr);
    ptr = mod(ptr, n) + 1;
end
rrPointer = ptr;
end
