import React, { useEffect, useMemo, useRef, useState, useCallback } from "react";
import ReactDOM from "react-dom/client";
import { AnimatePresence, motion } from "framer-motion";

/**
 * SAGE Face UI cinematic "robot face" for 5 states + error.
 * Demo: Space = next phase, D = toggle debug drawer, P = sample popup.
 */

// Constants
const WS_PING_INTERVAL = 7000;
const POPUP_DURATION = 1700;
const MORPH_TRANSITION_DURATION = 520;
const WS_RECONNECT_MIN = 500;
const WS_RECONNECT_MAX = 5000;
const WS_BACKOFF_MULTIPLIER = 2;

// Types
export type Phase = 'idle'|'listening'|'thinking'|'speaking'|'searching'|'error';
export const PHASES: Phase[] = ['idle','listening','thinking','speaking','searching','error'];

type UiMessage =
  | { type: "battery"; pct?: number; voltage?: number; is_charging?: boolean }
  | { type: "ui_state"; phase?: Phase; asr_partial?: string; asr_final?: string; llm_tokens?: number; tts_viseme?: number; error_msg?: string }
  | { type: "diag"; level?: "info"|"warn"|"error"; msg: string }
  | { type: "destination_set"; msg: string }
  | { type: "pong"; t: number }
  | { type: string; [k: string]: any };

const phasePalette: Record<Phase, { glow: string; accent: string; label: string }> = {
  idle:      { glow: "#6b7280", accent: "#94a3b8", label: "Idle" },
  listening: { glow: "#10b981", accent: "#34d399", label: "Listening" },
  thinking:  { glow: "#6366f1", accent: "#a5b4fc", label: "Thinking" },
  speaking:  { glow: "#f59e0b", accent: "#fde68a", label: "Speaking" },
  searching: { glow: "#06b6d4", accent: "#67e8f9", label: "Searching" },
  error:     { glow: "#ef4444", accent: "#fecaca", label: "Error" },
};

// Morph/pose variants per phase
const morphVariants: Record<Phase, any> = {
  idle:      { rotate: 0,   scale: 1.00, filter: "blur(0px)" },
  listening: { rotate: -6,  scale: 1.06, filter: "blur(0.3px)" },
  thinking:  { rotate: 8,   scale: 1.03, filter: "blur(0.35px)" },
  speaking:  { rotate: -4,  scale: 1.07, filter: "blur(0px)" },
  searching: { rotate: 10,  scale: 0.96, filter: "blur(0.25px)" },
  error:     { rotate: -12, scale: 0.94, filter: "blur(0.45px)" },
};

// Error Boundary Component
class ErrorBoundary extends React.Component<
  { children: React.ReactNode },
  { hasError: boolean; error: Error | null }
> {
  constructor(props: any) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error: Error) {
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: React.ErrorInfo) {
    console.error('SAGE Face Error:', error, errorInfo);
  }

  render() {
    if (this.state.hasError) {
      return (
        <div className="min-h-screen w-full bg-neutral-950 text-neutral-100 flex items-center justify-center">
          <div className="text-center space-y-4 p-8">
            <div className="text-6xl">�</div>
            <h1 className="text-2xl font-bold text-red-400">UI Error</h1>
            <p className="text-neutral-400">{this.state.error?.message || 'Unknown error'}</p>
            <button 
              onClick={() => window.location.reload()} 
              className="px-4 py-2 bg-neutral-800 hover:bg-neutral-700 rounded-lg transition"
            >
              Reload
            </button>
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}

function SageFaceInner(){
  // WS config - use env var or localhost fallback
  const WS_HOST = import.meta.env?.VITE_WS_HOST || location.hostname || "localhost";
  const WS_URL = `ws://${WS_HOST}:8765/ws/status`;

  const [connected, setConnected] = useState(false);
  const [latency, setLatency] = useState<number|null>(null);

  const [phase, setPhase] = useState<Phase>('idle');
  const [prevPhase, setPrevPhase] = useState<Phase>('idle');

  const [asrPartial, setAsrPartial] = useState('');
  const [asrFinal, setAsrFinal] = useState('');

  const [battery, setBattery] = useState({ pct: 0, charging: false });
  const [viseme, setViseme] = useState(0);

  const [drawerOpen, setDrawerOpen] = useState(false);
  const [popups, setPopups] = useState<{id:number; text:string; kind?:'info'|'ok'|'warn'|'error'}[]>([]);
  const idRef = useRef(1);
  const popupTimersRef = useRef<Map<number, number>>(new Map());

  // Memoized popup function with proper cleanup
  const pushPopup = useCallback((text: string, kind: 'info'|'ok'|'warn'|'error' = 'info') => {
    if (!text) return;
    const id = idRef.current++;
    setPopups(prev => [...prev, { id, text, kind }]);
    
    const timer = window.setTimeout(() => {
      setPopups(prev => prev.filter(p => p.id !== id));
      popupTimersRef.current.delete(id);
    }, POPUP_DURATION);
    
    popupTimersRef.current.set(id, timer);
  }, []);

  // Cleanup popup timers on unmount
  useEffect(() => {
    return () => {
      popupTimersRef.current.forEach(timer => clearTimeout(timer));
      popupTimersRef.current.clear();
    };
  }, []);

  // WebSocket with auto-reconnect + ping/pong latency
  useEffect(() => {
    let ws: WebSocket | null = null;
    let pingTimer: number | null = null;
    let pingTimestamps = new Map<number, number>();
    let backoff = WS_RECONNECT_MIN;
    let reconnectTimer: number | null = null;

    const connect = () => {
      try {
        ws = new WebSocket(WS_URL);

        ws.onopen = () => {
          setConnected(true);
          backoff = WS_RECONNECT_MIN;
          
          // Start ping every 7s
          pingTimer = window.setInterval(() => {
            if (ws && ws.readyState === WebSocket.OPEN) {
              const now = performance.now();
              pingTimestamps.set(now, now);
              ws.send(JSON.stringify({ type: "ping", t: now }));
              
              // Clean old timestamps (keep last 5)
              if (pingTimestamps.size > 5) {
                const oldest = Array.from(pingTimestamps.keys())[0];
                pingTimestamps.delete(oldest);
              }
            }
          }, WS_PING_INTERVAL);
        };

        ws.onmessage = (ev) => {
          let m: UiMessage;
          try { 
            m = JSON.parse(ev.data); 
          } catch (err) { 
            console.warn('Failed to parse WS message:', err);
            return; 
          }

          if (m.type === "pong" && typeof m.t === "number") {
            if (pingTimestamps.has(m.t)) {
              const rtt = Math.max(0, Math.round(performance.now() - m.t));
              setLatency(rtt);
              pingTimestamps.delete(m.t);
            }
            return;
          }
          
          if (m.type === "battery") {
            const pct = typeof m.pct === 'number' ? Math.max(0, Math.min(1, m.pct)) : 0;
            setBattery({ pct, charging: Boolean(m.is_charging) });
            return;
          }
          
          if (m.type === "ui_state") {
            if (m.phase && PHASES.includes(m.phase)) {
              setPhase(prev => {
                if (prev !== m.phase) {
                  setPrevPhase(prev);
                  return m.phase as Phase;
                }
                return prev;
              });
            }
            if (typeof m.asr_partial === "string") setAsrPartial(m.asr_partial);
            if (typeof m.asr_final === "string") setAsrFinal(m.asr_final);
            if (typeof m.tts_viseme === "number") setViseme(m.tts_viseme | 0);
            if (m.error_msg) {
              pushPopup(m.error_msg, 'error');
            }
            return;
          }
          
          if (m.type === "diag" && m.msg) {
            const kind = m.level === 'warn' ? 'warn' : m.level === 'error' ? 'error' : 'info';
            pushPopup(m.msg, kind);
          }
        };

        const onClose = () => {
          setConnected(false);
          setLatency(null);
          
          if (pingTimer) { 
            window.clearInterval(pingTimer); 
            pingTimer = null; 
          }
          
          pingTimestamps.clear();
          
          // Exponential backoff
          const wait = Math.min(backoff, WS_RECONNECT_MAX);
          backoff = Math.min(backoff * WS_BACKOFF_MULTIPLIER, WS_RECONNECT_MAX);
          
          reconnectTimer = window.setTimeout(connect, wait);
        };

        ws.onerror = (err) => {
          console.warn('WebSocket error:', err);
          onClose();
        };
        ws.onclose = onClose;
        
      } catch (err) {
        console.error('WebSocket connection failed:', err);
      }
    };

    connect();

    return () => {
      if (pingTimer) window.clearInterval(pingTimer);
      if (reconnectTimer) window.clearTimeout(reconnectTimer);
      if (ws) {
        try { ws.close(); } catch {}
      }
      pingTimestamps.clear();
    };
  }, [WS_URL, pushPopup]);

  // Keyboard shortcuts
  useEffect(() => {
    const handleKey = (e: KeyboardEvent) => {
      if (e.key === ' ') {
        e.preventDefault();
        setPhase(prev => {
          const idx = PHASES.indexOf(prev);
          const nextIdx = (idx + 1) % PHASES.length;
          setPrevPhase(prev);
          return PHASES[nextIdx];
        });
      } else if (e.key.toLowerCase() === 'd') {
        setDrawerOpen(v => !v);
      } else if (e.key.toLowerCase() === 'p') {
        pushPopup('Sample popup message', 'info');
      }
    };
    
    window.addEventListener('keydown', handleKey);
    return () => window.removeEventListener('keydown', handleKey);
  }, [pushPopup]);

  const pal = phasePalette[phase];

  return (
    <div className="min-h-screen w-full bg-neutral-950 text-neutral-100 overflow-hidden">
      <style>{FACE_CSS}</style>

      {/* HUD */}
      <div className="fixed top-3 right-4 flex items-center gap-3 z-30 select-none">
        <ConnPill ok={connected} latency={latency} />
        <MiniBattery pct={battery.pct} charging={battery.charging} />
      </div>

      {/* Debug nub */}
      <button 
        aria-label="Toggle debug drawer (D)" 
        onClick={() => setDrawerOpen(v => !v)} 
        className="fixed bottom-2 right-2 z-30 w-4 h-4 rounded-full bg-neutral-700/70 hover:bg-neutral-600/80 border border-neutral-600 transition-colors focus:outline-none focus:ring-2 focus:ring-neutral-500"
      />

      {/* Popups */}
      <div className="fixed top-16 right-4 z-30 space-y-2" role="alert" aria-live="polite">
        <AnimatePresence>
          {popups.map(p => <Popup key={p.id} text={p.text} kind={p.kind} />)}
        </AnimatePresence>
      </div>

      {/* FACE - Full screen */}
      <div className="fixed inset-0 flex items-center justify-center">
        <MorphStage phase={phase} prevPhase={prevPhase} viseme={viseme} />
      </div>

      <Drawer open={drawerOpen} onClose={() => setDrawerOpen(false)}>
        <div className="p-3 text-sm space-y-2">
          <div><strong>Phase:</strong> {pal.label}</div>
          <div><strong>Battery:</strong> {Math.round(battery.pct * 100)}%</div>
          <div><strong>Connected:</strong> {connected ? 'Yes' : 'No'}</div>
          {latency !== null && <div><strong>Latency:</strong> {latency}ms</div>}
          <div className="text-neutral-400 pt-2 space-y-1">
            <div><kbd className="kbd">Space</kbd> - Cycle phases (demo)</div>
            <div><kbd className="kbd">D</kbd> - Toggle drawer</div>
            <div><kbd className="kbd">P</kbd> - Sample popup</div>
          </div>
        </div>
      </Drawer>
    </div>
  );
}

function MorphStage({ phase, prevPhase, viseme }: { phase: Phase; prevPhase: Phase; viseme: number }) {
  const [transitioning, setTransitioning] = useState(false);
  const [displayPrev, setDisplayPrev] = useState<Phase>(prevPhase);
  const transitionTimerRef = useRef<number | null>(null);

  const palNow = phasePalette[phase];
  const palPrev = phasePalette[displayPrev];

  // Track transitions properly
  useEffect(() => {
    if (displayPrev === phase) return;
    
    setTransitioning(true);
    
    if (transitionTimerRef.current) {
      clearTimeout(transitionTimerRef.current);
    }
    
    transitionTimerRef.current = window.setTimeout(() => {
      setDisplayPrev(phase);
      setTransitioning(false);
      transitionTimerRef.current = null;
    }, MORPH_TRANSITION_DURATION);

    return () => {
      if (transitionTimerRef.current) {
        clearTimeout(transitionTimerRef.current);
      }
    };
  }, [phase, displayPrev]);

  const dPrev = phasePath(displayPrev);
  const dNow = phasePath(phase);

  return (
    <div className="face-container">
      {/* Background glow for idle/speaking - positioned behind everything */}
      {(phase === 'idle' || phase === 'speaking') && (
        <div 
          className="face-glow"
          style={{ 
            background: `radial-gradient(circle at center, ${hexA(palNow.accent, 0.12)} 0%, ${hexA(palNow.accent, 0.06)} 30%, transparent 60%)` 
          }} 
        />
      )}

      {/* Aura crossfade - centered properly */}
      <div className="aura-wrapper">
        <AnimatePresence mode="popLayout">
          <motion.div 
            key={`a-${displayPrev}`} 
            className={`aura aura-${displayPrev}`} 
            style={{ '--a': palPrev.glow } as any} 
            initial={{ opacity: 0.4 }} 
            animate={{ opacity: 0.0 }} 
            exit={{ opacity: 0.0 }} 
            transition={{ duration: 0.2 }} 
          />
        </AnimatePresence>
        <AnimatePresence mode="popLayout">
          <motion.div 
            key={`a-${phase}`} 
            className={`aura aura-${phase}`} 
            style={{ '--a': palNow.glow } as any} 
            initial={{ opacity: 0.0, scale: 0.98 }} 
            animate={{ opacity: 0.75, scale: 1 }} 
            exit={{ opacity: 0.0 }} 
            transition={{ type: 'spring', stiffness: 140, damping: 18 }} 
          />
        </AnimatePresence>
      </div>

      {/* Core panel */}
      <motion.div 
        className="face-core"
        initial={{ opacity: 0.0, scale: 0.985 }}
        animate={{ opacity: 1, scale: 1 }}
        transition={{ type: 'spring', stiffness: 180, damping: 22 }}
      >
        {/* Morph crossfade */}
        <div className="morph-stage" style={{ ['--accent' as any]: hexA(palNow.accent, 0.75) }}>
          <svg className="morph" viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet" aria-hidden="true">
            <defs>
              <radialGradient id="g1" cx="50%" cy="50%" r="60%">
                <stop offset="0%" stopColor="rgba(255,255,255,.65)" />
                <stop offset="100%" stopColor="rgba(255,255,255,.05)" />
              </radialGradient>
              <filter id="soft"><feGaussianBlur stdDeviation="1.1" /></filter>
              <filter id="mb"><feGaussianBlur stdDeviation="2" /></filter>
            </defs>
            
            {/* OUTGOING shape */}
            <AnimatePresence>
              {transitioning && (
                <motion.path 
                  key={`out-${displayPrev}`} 
                  d={dPrev} 
                  fill={displayPrev === 'idle' || displayPrev === 'speaking' ? 'url(#g1)' : hexA(phasePalette[displayPrev].accent, 0.12)} 
                  filter="url(#soft)"
                  initial={{ opacity: 1 }}
                  animate={{ opacity: 0 }}
                  exit={{ opacity: 0 }}
                  transition={{ duration: 0.48, ease: 'easeOut' }} 
                />
              )}
            </AnimatePresence>
            
            {/* INCOMING shape */}
            <motion.path 
              key={`in-${phase}`} 
              d={dNow} 
              fill={phase === 'idle' || phase === 'speaking' ? 'url(#g1)' : hexA(phasePalette[phase].accent, 0.10)} 
              filter="url(#soft)"
              initial={{ opacity: 0, scale: 0.94 }}
              animate={{ opacity: 1, scale: 1 }}
              transition={{ type: 'spring', stiffness: 240, damping: 22 }} 
            />
          </svg>

          {/* Dissolve/reveal layer */}
          <div className={`morph-reveal ${transitioning ? 'on' : ''}`} />
        </div>

        {/* State overlays */}
        <div className="state-overlay">
          {phase === 'speaking' && <SpeakingMouth viseme={viseme} />}
          {phase === 'thinking' && <ThinkingField />}
          {phase === 'listening' && <ListeningBeam />}
          {phase === 'searching' && <SearchingDial />}
          {phase === 'error' && <ErrorGlow />}
        </div>
      </motion.div>
    </div>
  );
}

function phasePath(p: Phase) {
  switch(p) {
    case 'idle':      return "M50,10 C72,10 90,28 90,50 C90,72 72,90 50,90 C28,90 10,72 10,50 C10,28 28,10 50,10 Z";
    case 'listening': return "M10,55 C20,25 80,25 90,55 C80,75 20,75 10,55 Z";
    case 'thinking':  return "M50,15 C70,15 85,35 85,50 C85,70 70,85 50,85 C30,85 15,70 15,50 C15,35 30,15 50,15 Z";
    case 'speaking':  return "M20,40 C35,30 65,30 80,40 C80,60 65,70 50,70 C35,70 20,60 20,40 Z";
    case 'searching': return "M40,20 C70,20 80,40 80,55 C80,75 60,85 45,85 C25,85 15,70 15,50 C15,35 25,20 40,20 Z";
    case 'error':     return "M20,20 L80,80 M80,20 L20,80";
  }
}

/** SPEAKING: Arc EQ mouth */
function SpeakingMouth({ viseme }: { viseme: number }) {
  const n = 24;
  const phases = useMemo(() => Array.from({length: n}, (_, i) => i * 0.38), [n]);
  const rafRef = useRef<number>(0);
  const [t, setT] = useState(0);
  
  useEffect(() => {
    let start: number | null = null;
    const loop = (ts: number) => {
      if (start === null) start = ts;
      setT((ts - start) / 1000);
      rafRef.current = requestAnimationFrame(loop);
    };
    rafRef.current = requestAnimationFrame(loop);
    
    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, []);

  const R = 70;

  return (
    <div className="overlay speaking">
      <svg className="arc-eq" style={{ overflow: 'visible' }} viewBox="-160 -160 320 240" aria-hidden="true">
        <defs>
          <linearGradient id="barGrad" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#fbbf24" />
            <stop offset="100%" stopColor="#f59e0b" />
          </linearGradient>
        </defs>
        <path d={`M ${-R} 0 A ${R} ${R} 0 0 1 ${R} 0`} className="arc-baseline" />
        {phases.map((phi, i) => {
          const a = (i / (n - 1)) * Math.PI;
          const cx = Math.cos(a) * R;
          const cy = Math.sin(a) * R;
          const base = 0.55 + 0.45 * Math.sin(t * 3.4 + phi);
          const focus = Math.max(0, 1 - Math.abs(((viseme % n) - i)) / 5);
          const amp = Math.min(1.2, base * 0.65 + focus * 0.5);
          const len = 16 + 36 * amp;
          const w = 4.2;
          
          return (
            <g key={i} transform={`translate(${cx} ${cy}) rotate(${(a * 180) / Math.PI})`}>
              <rect 
                x={-w / 2} 
                y={-len} 
                width={w} 
                height={len} 
                rx={2} 
                className={`bar-rect ${viseme % n === i ? 'bar-focus' : ''}`} 
                fill="url(#barGrad)" 
              />
            </g>
          );
        })}
      </svg>
    </div>
  );
}

/** THINKING: dynamic network */
function ThinkingField() {
  const [spark, setSpark] = useState(false);
  const [angles, setAngles] = useState({ a1: 0, a2: Math.PI / 3, a3: Math.PI * 0.75, a4: Math.PI * 1.2 });
  const rafRef = useRef<number>(0);
  const sparkTimerRef = useRef<number>(0);

  useEffect(() => {
    let last = 0;
    const step = (ts: number) => {
      if (!last) last = ts;
      const dt = Math.min(0.032, (ts - last) / 1000);
      last = ts;
      
      setAngles(a => ({
        a1: (a.a1 + dt * 0.9) % (Math.PI * 2),
        a2: (a.a2 - dt * 0.65 + Math.sin(ts / 1200) * 0.0005) % (Math.PI * 2),
        a3: (a.a3 + dt * 0.48) % (Math.PI * 2),
        a4: (a.a4 - dt * 0.34) % (Math.PI * 2),
      }));
      
      rafRef.current = requestAnimationFrame(step);
    };
    
    rafRef.current = requestAnimationFrame(step);
    
    const scheduleSpark = () => {
      setSpark(true);
      setTimeout(() => setSpark(false), 200);
      sparkTimerRef.current = window.setTimeout(scheduleSpark, 1000 + Math.random() * 600);
    };
    scheduleSpark();

    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
      if (sparkTimerRef.current) clearTimeout(sparkTimerRef.current);
    };
  }, []);

  const R1 = 34, R2 = 56;
  const nodes = useMemo(() => [
    { r: R2, key: 'n1', hue: '#a5b4fc' },
    { r: R1, key: 'n2', hue: '#c4b5fd' },
    { r: R2, key: 'n3', hue: '#93c5fd' },
    { r: R1, key: 'n4', hue: '#a7f3d0' },
  ], []);
  
  const angs = [angles.a1, angles.a2, angles.a3, angles.a4];
  const pts = angs.map((ang, i) => ({ 
    x: Math.cos(ang) * nodes[i].r, 
    y: Math.sin(ang) * nodes[i].r 
  }));

  function qPath(p: any, q: any, bulge = 0.28) {
    const mx = (p.x + q.x) / 2, my = (p.y + q.y) / 2;
    const nx = -(q.y - p.y), ny = (q.x - p.x);
    const len = Math.hypot(nx, ny) || 1;
    const ux = nx / len, uy = ny / len;
    const cx = mx + ux * bulge * 12, cy = my + uy * bulge * 12;
    return `M ${p.x.toFixed(2)} ${p.y.toFixed(2)} Q ${cx.toFixed(2)} ${cy.toFixed(2)} ${q.x.toFixed(2)} ${q.y.toFixed(2)}`;
  }

  return (
    <div className="thinking">
      <svg className="net" viewBox="-80 -80 160 160" aria-hidden="true">
        <path d={qPath(pts[0], pts[1], 0.22)} className="strand s1" />
        <path d={qPath(pts[1], pts[2], 0.30)} className="strand s2" />
        <path d={qPath(pts[2], pts[3], 0.24)} className="strand s3" />
        <path d={qPath(pts[3], pts[0], 0.28)} className="strand s4" />
        <g className="ring">
          <circle r="60" className="ring-a" />
          <circle r="44" className="ring-b" />
        </g>
        {pts.map((p, i) => (
          <circle key={i} cx={p.x} cy={p.y} r="3" className="node-dot" />
        ))}
      </svg>
      {spark && <span className="spark" />}
    </div>
  );
}

/** LISTENING: concentric ripples */
function ListeningBeam() {
  return (
    <motion.div 
      className="listening"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0.9 }}
      transition={{ duration: 0.18 }}
    >
      <div className="pulse-core" />
      <span className="ring r1" />
      <span className="ring r2" />
      <span className="ring r3" />
      <div className="beam" />
    </motion.div>
  );
}

/** SEARCHING: radar dial */
function SearchingDial() {
  const [ang, setAng] = useState(0);
  const [ping, setPing] = useState<number | null>(null);
  const sweepTimerRef = useRef<number>(0);
  const pingTimerRef = useRef<number>(0);
  
  useEffect(() => {
    sweepTimerRef.current = window.setInterval(() => setAng(a => (a + 8) % 360), 80);
    pingTimerRef.current = window.setInterval(() => setPing(Math.random() * 360), 800);
    
    return () => {
      if (sweepTimerRef.current) clearInterval(sweepTimerRef.current);
      if (pingTimerRef.current) clearInterval(pingTimerRef.current);
    };
  }, []);
  
  return (
    <div className="search-dial">
      <div className="sweep" style={{ transform: `rotate(${ang}deg)` }} />
      <div className="ticks" />
      {ping !== null && <div className="ping" style={{ transform: `rotate(${ping}deg)` }} />}
    </div>
  );
}

function ErrorGlow() { 
  return <div className="errx fine"><div className="glitch" /></div>;
}

// HUD Components
function ConnPill({ ok, latency }: { ok: boolean; latency: number | null }) {
  return (
    <div className="hud-pill" role="status" aria-live="polite">
      <span className={`dot ${ok ? 'ok' : 'bad'}`} />
      <span>
        {ok ? 'Connected' : 'Disconnected'}
        {ok && latency !== null ? ` � ${latency}ms` : ''}
      </span>
    </div>
  );
}

function MiniBattery({ pct, charging }: { pct: number; charging: boolean }) {
  const percentage = Math.round(pct * 100);
  const barWidth = Math.max(0, Math.min(14, (pct * 14)));
  
  return (
    <div className="hud-pill" role="status" aria-label={`Battery ${percentage}% ${charging ? 'charging' : ''}`}>
      <svg width="18" height="10" viewBox="0 0 18 10" aria-hidden="true">
        <rect x="1" y="1" width="14" height="8" rx="2" ry="2" fill="none" stroke="#666" strokeWidth="1" />
        <rect x="15" y="3" width="2" height="4" rx="1" fill="#666" />
        <rect x="2" y="2" width={barWidth} height="6" fill="#a3e635" />
      </svg>
      <span>
        {isFinite(percentage) ? `${percentage}%` : '--%'} {charging ? '�' : ''}
      </span>
    </div>
  );
}

function Popup({ text, kind = 'info' }: { text: string; kind?: 'info' | 'ok' | 'warn' | 'error' }) {
  const colorMap: Record<'info' | 'ok' | 'warn' | 'error', string> = {
    info: '#94a3b8',
    ok: '#34d399',
    warn: '#f59e0b',
    error: '#f87171'
  };
  
  return (
    <motion.div 
      className="popup" 
      style={{ ['--c' as any]: colorMap[kind] }}
      initial={{ opacity: 0, y: -8 }}
      animate={{ opacity: 1, y: 0 }}
      exit={{ opacity: 0, y: -8 }}
      transition={{ duration: 0.2 }}
    >
      {text}
    </motion.div>
  );
}

function Drawer({ open, onClose, children }: { open: boolean; onClose: () => void; children: React.ReactNode }) {
  // Close on Escape key
  useEffect(() => {
    if (!open) return;
    
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    };
    
    window.addEventListener('keydown', handleEscape);
    return () => window.removeEventListener('keydown', handleEscape);
  }, [open, onClose]);
  
  // Focus trap
  const drawerRef = useRef<HTMLDivElement>(null);
  
  useEffect(() => {
    if (open && drawerRef.current) {
      const focusable = drawerRef.current.querySelectorAll('button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])');
      if (focusable.length > 0) {
        (focusable[0] as HTMLElement).focus();
      }
    }
  }, [open]);
  
  return (
    <div className={`drawer ${open ? 'open' : ''}`} role="dialog" aria-modal="true" aria-label="Debug drawer">
      <div className="drawer-backdrop" onClick={onClose} aria-hidden="true" />
      <div className="drawer-body" ref={drawerRef}>
        <div className="flex justify-between items-center px-3 py-2 border-b border-neutral-800/70">
          <div className="text-neutral-300 text-sm font-semibold">Debug</div>
          <button 
            className="text-neutral-400 hover:text-neutral-200 text-sm px-2 py-1 rounded hover:bg-neutral-800 transition focus:outline-none focus:ring-2 focus:ring-neutral-500" 
            onClick={onClose}
            aria-label="Close drawer"
          >
            Close
          </button>
        </div>
        <div className="p-2">{children}</div>
      </div>
    </div>
  );
}

// Utility functions
function hexA(hex: string, a: number) {
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  return `rgba(${r},${g},${b},${a})`;
}

// Main export wrapped in error boundary
export default function SageFace() {
  return (
    <ErrorBoundary>
      <SageFaceInner />
    </ErrorBoundary>
  );
}

// CSS
const FACE_CSS = `
:root { --bg: #0a0a0a; }
html, body { background: var(--bg); margin: 0; padding: 0; }

/* Keyboard indicator */
.kbd {
  display: inline-block;
  padding: 0.15rem 0.4rem;
  font-size: 0.75rem;
  font-family: ui-monospace, monospace;
  background: #262626;
  border: 1px solid #404040;
  border-radius: 0.25rem;
  box-shadow: 0 1px 2px rgba(0,0,0,0.3);
}

/* HUD */
.hud-pill {
  display: inline-flex;
  align-items: center;
  gap: 0.45rem;
  font: 600 12px system-ui;
  color: #d1d5db;
  background: rgba(17, 17, 17, 0.7);
  border: 1px solid #3f3f46;
  padding: 0.35rem 0.5rem;
  border-radius: 9999px;
  backdrop-filter: blur(6px);
}

.dot {
  width: 8px;
  height: 8px;
  border-radius: 9999px;
  background: #ef4444;
  box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.4);
  animation: pulse 2.4s infinite;
}

.dot.ok {
  background: #10b981;
  box-shadow: 0 0 0 0 rgba(16, 185, 129, 0.4);
}

.dot.bad {
  background: #ef4444;
  box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.4);
}

@keyframes pulse {
  0% { box-shadow: 0 0 0 0 rgba(255, 255, 255, 0.1); }
  70% { box-shadow: 0 0 0 10px rgba(255, 255, 255, 0); }
  100% { box-shadow: 0 0 0 0 rgba(255, 255, 255, 0); }
}

/* Drawer */
.drawer {
  position: fixed;
  inset: 0;
  pointer-events: none;
  z-index: 50;
}

.drawer.open {
  pointer-events: auto;
}

.drawer-backdrop {
  position: absolute;
  inset: 0;
  background: rgba(0, 0, 0, 0.3);
  opacity: 0;
  transition: opacity 0.2s;
}

.drawer.open .drawer-backdrop {
  opacity: 1;
}

.drawer-body {
  position: absolute;
  right: 0;
  top: 0;
  bottom: 0;
  width: min(420px, 90vw);
  transform: translateX(100%);
  transition: transform 0.25s ease;
  background: #0b0b0b;
  border-left: 1px solid #27272a;
  box-shadow: -30px 0 60px rgba(0, 0, 0, 0.35);
}

.drawer.open .drawer-body {
  transform: translateX(0);
}

/* Popups */
.popup {
  background: rgba(17, 24, 39, 0.9);
  color: #e5e7eb;
  border: 1px solid rgba(255, 255, 255, 0.08);
  border-left: 4px solid var(--c);
  padding: 0.6rem 0.75rem;
  border-radius: 0.75rem;
  backdrop-filter: blur(6px);
  box-shadow: 0 8px 30px rgba(0, 0, 0, 0.35);
}

/* Face */
.face-container {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  display: flex;
  align-items: center;
  justify-content: center;
  width: 100vmin;
  height: 100vmin;
}

.face-glow {
  position: absolute;
  left: 0;
  top: 0;
  width: 100%;
  height: 100%;
  border-radius: 9999px;
  pointer-events: none;
  z-index: 0;
}

.aura-wrapper {
  position: absolute;
  left: 0;
  top: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
  z-index: 1;
}

.face-core {
  position: relative;
  display: flex;
  align-items: center;
  justify-content: center;
  width: 85%;
  height: 85%;
  border-radius: 32px;
  border: 1px solid #262626;
  background: radial-gradient(120% 160% at 50% 50%, rgba(255, 255, 255, 0.02), rgba(0, 0, 0, 0.24));
  box-shadow: inset 0 0 120px rgba(255, 255, 255, 0.02), 0 24px 80px rgba(0, 0, 0, 0.45);
  overflow: hidden;
  z-index: 2;
}

.morph-stage {
  position: relative;
  width: 100%;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
}

/* Metamorphosis reveal */
.morph-reveal {
  position: absolute;
  inset: 0;
  pointer-events: none;
  mask-image: radial-gradient(circle at 50% 50%, rgba(0, 0, 0, 1) 35%, rgba(0, 0, 0, 0) 64%);
  background: conic-gradient(from 0deg, rgba(255, 255, 255, 0.06), rgba(255, 255, 255, 0) 45%, rgba(255, 255, 255, 0.08) 60%, rgba(255, 255, 255, 0));
  opacity: 0;
  transform: scale(0.96);
  transition: opacity 0.36s ease, transform 0.36s ease;
  mix-blend-mode: overlay;
}

.morph-reveal.on {
  opacity: 0.7;
  transform: scale(1) rotate(8deg);
  animation: swirl 0.52s ease-out;
}

@keyframes swirl {
  from { filter: blur(1.4px); }
  to { filter: blur(0px); }
}

/* Morph shape */
.morph {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  width: min(58vmin, 680px);
  height: min(58vmin, 680px);
  pointer-events: none;
  z-index: 1;
}

/* Overlays */
.overlay {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  z-index: 3;
  pointer-events: none;
}

.state-overlay {
  position: absolute;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 3;
  pointer-events: none;
}

/* Speaking (Arc EQ) */
.arc-eq {
  width: min(70vmin, 720px);
  height: auto;
  overflow: visible;
  filter: drop-shadow(0 14px 36px rgba(245, 158, 11, 0.2));
}

.arc-baseline {
  fill: none;
  stroke: rgba(253, 230, 138, 0.35);
  stroke-width: 1.5;
}

.bar-rect {
  fill: url(#barGrad);
}

.bar-focus {
  filter: drop-shadow(0 0 12px rgba(245, 158, 11, 0.55));
}

/* Listening */
.listening {
  position: relative;
  width: min(58vmin, 720px);
  height: min(58vmin, 720px);
  display: flex;
  align-items: center;
  justify-content: center;
}

.pulse-core {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  width: 24%;
  height: 24%;
  border-radius: 9999px;
  box-shadow: inset 0 0 30px rgba(255, 255, 255, 0.08);
  background: radial-gradient(60% 60% at 50% 50%, rgba(255, 255, 255, 0.10), rgba(255, 255, 255, 0) 70%),
    conic-gradient(from 0deg, rgba(255, 255, 255, 0.12), rgba(255, 255, 255, 0) 40%, rgba(255, 255, 255, 0.12) 80%, rgba(255, 255, 255, 0));
  animation: pulseSpin 2.2s ease-in-out infinite;
}

@keyframes pulseSpin {
  0% { transform: translate(-50%, -50%) scale(0.95) rotate(0deg); }
  50% { transform: translate(-50%, -50%) scale(1.05) rotate(8deg); }
  100% { transform: translate(-50%, -50%) scale(0.95) rotate(0deg); }
}

.ring {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  width: 40%;
  height: 40%;
  border-radius: 9999px;
  border: 2px solid rgba(255, 255, 255, 0.10);
  box-shadow: 0 0 30px rgba(255, 255, 255, 0.04) inset;
}

.r1 { animation: listenPulse 1.6s ease-out infinite; }
.r2 { animation: listenPulse 1.6s ease-out 0.25s infinite; }
.r3 { animation: listenPulse 1.6s ease-out 0.5s infinite; }

@keyframes listenPulse {
  0% { opacity: 0.45; transform: translate(-50%, -50%) scale(0.75); }
  60% { opacity: 0.18; }
  100% { opacity: 0; transform: translate(-50%, -50%) scale(1.35); }
}

.beam {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  width: 86%;
  height: 10px;
  border-radius: 9999px;
  background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.22), transparent);
  background-size: 200% 100%;
  filter: blur(1px);
  animation: beamSweep 2.4s linear infinite, beamBreathe 1.8s ease-in-out infinite;
}

@keyframes beamSweep {
  from { background-position: 0 0; }
  to { background-position: 200% 0; }
}

@keyframes beamBreathe {
  0%, 100% { transform: translate(-50%, -50%) scaleX(0.94); }
  50% { transform: translate(-50%, -50%) scaleX(1.06); }
}

/* Thinking */
.thinking {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  width: min(58vmin, 720px);
  height: min(58vmin, 720px);
  border-radius: 9999px;
}

.thinking .net {
  width: 100%;
  height: 100%;
}

.strand {
  fill: none;
  stroke: rgba(165, 180, 252, 0.35);
  stroke-width: 1.4;
  filter: drop-shadow(0 0 10px rgba(165, 180, 252, 0.16));
}

.s2 { stroke: rgba(103, 232, 249, 0.35); }
.s3 { stroke: rgba(167, 243, 208, 0.35); }
.s4 { stroke: rgba(180, 198, 255, 0.35); }

.ring {
  transform-origin: 50% 50%;
  animation: ringSpin 12s linear infinite;
}

.ring-a {
  fill: none;
  stroke: rgba(165, 180, 252, 0.22);
  stroke-width: 1;
}

.ring-b {
  fill: none;
  stroke: rgba(165, 180, 252, 0.16);
  stroke-width: 1;
  animation: ringSpinR 16s linear infinite;
  transform-origin: 50% 50%;
}

@keyframes ringSpin {
  to { transform: rotate(360deg); }
}

@keyframes ringSpinR {
  to { transform: rotate(-360deg); }
}

.node-dot {
  fill: #a5b4fc;
  filter: drop-shadow(0 0 8px rgba(165, 180, 252, 0.8));
}

.spark {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 6px;
  height: 6px;
  margin: -3px 0 0 -3px;
  border-radius: 9999px;
  background: #a5b4fc;
  box-shadow: 0 0 20px rgba(165, 180, 252, 0.9);
  animation: sparkFlash 0.2s ease-out;
}

@keyframes sparkFlash {
  from { opacity: 1; transform: scale(1); }
  to { opacity: 0; transform: scale(2); }
}

/* Searching */
.search-dial {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  width: min(30vmin, 240px);
  aspect-ratio: 1;
  border-radius: 50%;
  border: 1px solid rgba(255, 255, 255, 0.08);
  background: none;
  box-shadow: inset 0 0 40px rgba(0, 0, 0, 0.25);
}

.search-dial .ticks {
  position: absolute;
  inset: 12%;
  border-radius: 50%;
  background: radial-gradient(circle at 50% 50%, rgba(255, 255, 255, 0.06) 2px, transparent 3px) 0 0 / 18px 18px;
  opacity: 0.25;
}

.search-dial .sweep {
  position: absolute;
  inset: 10%;
  border-radius: 50%;
  background: conic-gradient(#67e8f9 0 14deg, transparent 14deg 360deg);
  mix-blend-mode: screen;
  filter: blur(0.2px);
}

.search-dial .ping {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  margin: auto;
  width: 4px;
  height: 4px;
  border-radius: 9999px;
  background: #67e8f9;
  transform-origin: 50% 50%;
  box-shadow: 0 0 16px currentColor;
}

.search-dial .ping::after {
  content: "";
  position: absolute;
  inset: -6px;
  border-radius: 9999px;
  border: 1px solid currentColor;
  animation: pingFade 0.6s ease-out 1;
}

@keyframes pingFade {
  from { opacity: 0.6; transform: scale(0.9); }
  to { opacity: 0; transform: scale(1.4); }
}

/* Error */
.errx {
  position: absolute;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  width: min(36vmin, 300px);
  height: min(36vmin, 300px);
}

.errx::before,
.errx::after {
  content: "";
  position: absolute;
  inset: 20% 48%;
  background: #ef4444;
  border-radius: 4px;
  box-shadow: 0 0 24px rgba(239, 68, 68, 0.35);
}

.errx::before {
  transform: rotate(45deg);
}

.errx::after {
  transform: rotate(-45deg);
}

.errx.fine .glitch {
  position: absolute;
  inset: 0;
  background: linear-gradient(transparent 40%, rgba(255, 255, 255, 0.06) 41%, transparent 42%);
  animation: glitch 0.06s steps(1) 1;
  pointer-events: none;
}

@keyframes glitch {
  0% { clip-path: inset(0 0 80% 0); }
  50% { clip-path: inset(60% 0 0 0); }
  100% { clip-path: inset(0 0 0 0); }
}

/* Auras */
.aura {
  position: absolute;
  left: 0;
  top: 0;
  width: 100%;
  height: 100%;
  filter: blur(60px);
  border-radius: 9999px;
  opacity: 0.7;
  transition: filter 0.4s ease, opacity 0.4s ease;
}

.aura::before {
  content: "";
  position: absolute;
  left: 0;
  top: 0;
  width: 100%;
  height: 100%;
  border-radius: 9999px;
  background: radial-gradient(circle at 50% 50%, var(--a) 0%, transparent 60%);
  opacity: 0.9;
}

.aura-listening {
  animation: auraPulse 1.9s ease-in-out infinite;
}

.aura-thinking {
  animation: auraPulse 2.4s ease-in-out infinite;
  filter: blur(64px);
}

.aura-speaking {
  animation: auraPulse 1.2s ease-in-out infinite;
}

.aura-searching {
  animation: auraPulse 1.4s ease-in-out infinite;
}

.aura-error {
  animation: shake 0.7s ease-in-out both infinite;
  opacity: 0.8;
}

@keyframes auraPulse {
  0%, 100% { transform: scale(0.96); }
  50% { transform: scale(1.04); }
}

@keyframes shake {
  10%, 90% { transform: translateX(-1px); }
  20%, 80% { transform: translateX(2px); }
  30%, 50%, 70% { transform: translateX(-4px); }
  40%, 60% { transform: translateX(4px); }
}

/* Caret */
@keyframes caret {
  0%, 100% { box-shadow: inset -1px 0 0 rgba(255, 255, 255, 0.0); }
  50% { box-shadow: inset -1px 0 0 rgba(255, 255, 255, 0.9); }
}

.animate-caret {
  padding-right: 2px;
  animation: caret 1s steps(1) infinite;
}

/* Lock viewport: no page scroll flicker */
html, body, #root {
  height: 100%;
  overflow: hidden;          /* ← key line */
  overscroll-behavior: none; /* avoids bounce on touch devices */
}

/* Contain animated glows so they can't trigger layout scroll */
.face-container,
.aura-wrapper,
.face-glow {
  overflow: hidden;
}

/* If the drawer content is tall, let *it* scroll internally */
.drawer-body {
  overflow: auto;
}
  
/* Reduced motion */
@media (prefers-reduced-motion: reduce) {
  .aura,
  .listening,
  .arc-eq,
  .ring-a,
  .ring-b,
  .beam,
  .sweep,
  .pulse-core {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
  }
  
  .aura-error {
    animation: none;
  }
}
`;

// Mount
const root = document.getElementById("root");
if (root) {
  ReactDOM.createRoot(root).render(<SageFace />);
}