function state = step_traffic(state, slot, cfg)
% STEP_TRAFFIC  Generates video frames for the dual-source model.
%
%  Two regimes are supported via cfg.videoModel:
%
%    'atomic' (draft Sec. II-D premise)
%       "each frame is an atomic update of uniform semantics value,
%        indexed solely by its generation timestamp t_{f_i}."
%       Every frame is a single-packet update (L_vid = 1), generated
%       at rate cfg.fps. The intra-GOP structure is abstracted away,
%       so the video source behaves as a rate-cfg.fps stream of
%       uniform atomic updates — the regime for which R_vid (eq. 17)
%       is defined.
%
%    'gop'
%       Legacy behaviour: intra-GOP structure with I-frames of size
%       cfg.L_I packets and P-frames of size cfg.L_P packets, within
%       groups of cfg.GOP frames. Retained as a surrogate for
%       realistic H.265 traffic in extended experiments.
%
%  In both modes the single-update buffer semantics are preserved:
%  a new frame replaces any unfinished frame, so only the freshest
%  frame is in flight at any time (generate-at-will, per Kadota).
%
%  The C2 / telemetry source is always "generate-at-will" with
%  L = cfg.L_c2 = 1 and requires no explicit state here; its
%  delivery is handled directly in step_transmit.

useAtomic = strcmp(cfg.videoModel, 'atomic');

for u = state.activeDrones
    % WHILE loop: consume all due frame intervals this slot
    while slot >= state.dual(u).next_frame_slot
        state.dual(u).frame_counter = state.dual(u).frame_counter + 1;

        if useAtomic
            newL = cfg.L_vid;
        else
            if mod(state.dual(u).frame_counter - 1, cfg.GOP) == 0
                newL = cfg.L_I;
            else
                newL = cfg.L_P;
            end
        end

        % Single-update PENDING buffer: newest frame overwrites old pending.
        % The IN-FLIGHT frame (vid_remaining) is NOT interrupted.
        state.dual(u).vid_pending     = true;
        state.dual(u).vid_pending_slot = slot;
        state.dual(u).vid_pending_L   = newL;
        state.dual(u).next_frame_slot  = state.dual(u).next_frame_slot + cfg.slots_per_frame;
    end

    % If nothing is in-flight, load pending frame immediately
    if state.dual(u).vid_remaining == 0 && state.dual(u).vid_pending
        state.dual(u).vid_remaining = state.dual(u).vid_pending_L;
        state.dual(u).vid_gen_slot  = state.dual(u).vid_pending_slot;
        state.dual(u).vid_L         = state.dual(u).vid_pending_L;
        state.dual(u).vid_pending   = false;
    end
    % C2: generate-at-will — always has a fresh packet available.
    % No explicit state needed; delivery is handled in step_transmit.
end
end
