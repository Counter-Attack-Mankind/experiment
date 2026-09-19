function [a_s, b_s, c_s, d_s] = EstimateScaledAABB(step_len, k)
% Scheme 3 uses the full embodied footprint in Hybrid A*.
% Keep this local helper to avoid hidden scale fallback from shared code.
[a_s, b_s, c_s, d_s] = EstimateAABB(step_len, k);
end
