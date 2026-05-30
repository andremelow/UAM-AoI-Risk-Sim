function update_dashboard(db, state, scene, txSlots, slot, cfg)
% UPDATE_DASHBOARD  Atualização por frame — sem 3D, sem PL, painel separado.

N = cfg.numDrones;

% ── Limpa drones que aterraram ───────────────────────────────────────
for i=1:N
    if slot > state.endSlots(i) && ~state.pendingMsg{i}.valid
        set(db.hUnc{i},'XData',[],'YData',[]); set(db.hMap{i},'XData',[],'YData',[]);
        set(db.hVid{i},'XData',[],'YData',[]); set(db.hH1{i},'XData',[],'YData',[]);
        set(db.hH2{i},'XData',[],'YData',[]);
        set(db.hMapMarker(i),'XData',NaN,'YData',NaN);
        set(db.hMapTrail(i),'XData',NaN,'YData',NaN);
        set(db.hMapAoiCirc(i),'XData',NaN,'YData',NaN);
        set(db.hSNR(i),'XData',[],'YData',[]);
    end
end

% ── Médias e R_sys ───────────────────────────────────────────────────
set(db.hUncMean,'XData',state.uncMeanT,'YData',state.uncMeanV);
set(db.hMapMean,'XData',state.mapMeanT,'YData',state.mapMeanV);
set(db.hVidMean,'XData',state.vidMeanT,'YData',state.vidMeanV);
set(db.hH1Mean, 'XData',state.h1MeanT, 'YData',state.h1MeanV);
set(db.hH2Mean, 'XData',state.h2MeanT, 'YData',state.h2MeanV);
set(db.hRsys,   'XData',state.rSysTime,'YData',state.rSysData);

theta = linspace(0,2*pi,40);

% ── Loop por drone ───────────────────────────────────────────────────
for i=state.activeDrones
    pos  = state.pendingMsg{i}.pos;
    posN = pos(1);  posE = pos(2);
    snr_i = state.snrLast(i);

    % Trail de posição (últimos 150 pontos)
    db.posNorthHist{i}(end+1) = posN;
    db.posEastHist{i}(end+1)  = posE;
    if numel(db.posNorthHist{i}) > 150
        db.posNorthHist{i} = db.posNorthHist{i}(end-149:end);
        db.posEastHist{i}  = db.posEastHist{i}(end-149:end);
    end
    set(db.hMapTrail(i),'XData',db.posEastHist{i},'YData',db.posNorthHist{i});

    % Marcador de posição actual
    set(db.hMapMarker(i),'XData',posE,'YData',posN);

    % Círculo de incerteza proporcional a AoI
    if ~isempty(state.h1Val{i})
        h1_s = state.h1Val{i}(end) / cfg.updateRate;   % slots → s for radius
    else
        h1_s = 0;
    end
    r_aoi = cfg.r_min + cfg.v_max * h1_s;
    set(db.hMapAoiCirc(i),'XData',posE+r_aoi*cos(theta),'YData',posN+r_aoi*sin(theta));

    % SNR vs North — usa histórico acumulado externamente (step 6f no run_sim)
    if ~isempty(db.snrNorthHist{i})
        set(db.hSNR(i),'XData',db.snrNorthHist{i},'YData',db.snrValHist{i});
    end

    % Risk + AoI plots
    set(db.hUnc{i},'XData',state.riskTime{i},'YData',state.riskUnc{i});
    set(db.hMap{i},'XData',state.riskTime{i},'YData',state.riskMap{i});
    set(db.hVid{i},'XData',state.riskTime{i},'YData',state.riskVid{i});
    set(db.hH1{i}, 'XData',state.h1Time{i},  'YData',state.h1Val{i});
    set(db.hH2{i}, 'XData',state.h2Time{i},  'YData',state.h2Val{i});
end

% ── Barra de status ──────────────────────────────────────────────────
accentWarn=[1.0 0.75 0.20]; accentDanger=[1.0 0.25 0.25]; green=[0.30 0.90 0.45];
activeMask=arrayfun(@(k) state.pendingMsg{k}.valid, 1:N);
nAct=sum(activeMask);
set(db.hTileVal(db.IDX_ACTIVE),'String',sprintf('%d / %d',nAct,N));
set(db.hTileVal(db.IDX_SCHED), 'String',sprintf('%d / slot  (%d src)',cfg.dronesPerSlot,numel(txSlots)));
if nAct>0
    if ~isempty(state.h1MeanV), curH1=state.h1MeanV(end); else, curH1=0; end
    set(db.hTileVal(db.IDX_H1),'String',fmt_slots(curH1),'ForegroundColor',pick_c(curH1,2,5,green,accentWarn,accentDanger));
    if ~isempty(state.h2MeanV), curH2=state.h2MeanV(end); else, curH2=0; end
    set(db.hTileVal(db.IDX_H2),'String',fmt_slots(curH2),'ForegroundColor',pick_c(curH2,2,10,green,accentWarn,accentDanger));
    if ~isempty(state.vidMeanV), curRv=state.vidMeanV(end); else, curRv=0; end
    set(db.hTileVal(db.IDX_RVID),'String',sprintf('%.2f',curRv),'ForegroundColor',pick_c(curRv,1,10,green,accentWarn,accentDanger));
    if ~isempty(state.rSysData)
        rs=state.rSysData(end);
        if rs<0.5, lv='LOW'; lc=green; elseif rs<0.85, lv='MODERATE'; lc=accentWarn;
        elseif rs<1.0, lv='HIGH'; lc=[1.0 0.50 0.10]; else, lv='CRITICAL'; lc=accentDanger; end
        set(db.hTileVal(db.IDX_RSYS),'String',sprintf('%.1f',rs),'ForegroundColor',lc);
        set(db.hTileVal(db.IDX_LEVEL),'String',lv,'ForegroundColor',lc);
    end
end
set(db.hTileVal(db.IDX_TIME),'String',sprintf('slot %d',slot));

% ── Janela separada: Drone Status ────────────────────────────────────
if ishandle(db.fig_status)
    nAct = sum(arrayfun(@(k) state.pendingMsg{k}.valid, 1:N));
    header  = sprintf('  Slot: %d  |  %d/%d drones', slot, nAct, N);
    divider = '  ------+---------+------+---------------+---------------+------';
    colhdr  = '  UAV   | AoI[sl] |  SNR | C2  ok/fail   | Vid  ok/fail  | Frms ';
    lines = [{header}; {colhdr}; {divider}];
    for k=1:N
        if state.pendingMsg{k}.valid
            h1_sl = 0; if ~isempty(state.h1Val{k}), h1_sl=state.h1Val{k}(end); end
            snr_k = state.snrLast(k);
            if isnan(snr_k), snr_s=' N/A '; else, snr_s=sprintf('%+.0fdB',snr_k); end
            c2ok  = state.slotsC2(k);
            c2fl  = state.slotsC2Fail(k);
            vok   = state.slotsVid(k);
            vfl   = state.slotsVidFail(k);
            nfrm  = state.dual(k).n_delivered;
            row = sprintf('  UAV%2d | %5.0fms | %s | C2:%4d/%4d | Vid:%4d/%4d | Frm:%4d', ...
                k, h1_sl, snr_s, c2ok, c2fl, vok, vfl, nfrm);
        elseif slot < state.startSlots(k)
            row = sprintf('  UAV%2d | [waiting  slot %d]', k, state.startSlots(k));
        else
            row = sprintf('  UAV%2d | [pousado]', k);
        end
        lines{end+1} = row; %#ok<AGROW>
    end
    set(db.hDroneList,'String',lines);
end

drawnow limitrate;
end

function c=pick_c(v,lo,hi,cG,cW,cD)
    if v<lo, c=cG; elseif v<hi, c=cW; else, c=cD; end
end

function s=fmt_slots(v)
% FMT_SLOTS  Compact integer display for slot counts.
n = round(v);
if     n < 1000,  s = sprintf('%d sl', n);
elseif n < 10000, s = sprintf('%.1fk sl', n/1000);
else,             s = sprintf('%dk sl', round(n/1000));
end
end
