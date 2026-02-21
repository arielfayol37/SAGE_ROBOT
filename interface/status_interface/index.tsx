import React, { useEffect, useMemo, useRef, useState, useCallback } from "react";
import { AnimatePresence, motion } from "framer-motion";
import ReactDOM from "react-dom/client";
// Note: Import framer-motion in your package.json if not already:
// npm install framer-motion

/**
 * SAGE Orb UI - Unified spherical interface for robot states
 * Demo: Space = next phase, D = toggle debug drawer, P = sample popup
 */

// Constants
const WS_PING_INTERVAL = 7000;
const POPUP_DURATION = 1700;
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

const phasePalette: Record<Phase, { glow: string; accent: string; inner: string; label: string }> = {
  idle:      { glow: "#6b7280", accent: "#94a3b8", inner: "#e2e8f0", label: "Idle" },
  listening: { glow: "#10b981", accent: "#34d399", inner: "#d1fae5", label: "Listening" },
  thinking:  { glow: "#6366f1", accent: "#a5b4fc", inner: "#e0e7ff", label: "Thinking" },
  speaking:  { glow: "#f59e0b", accent: "#fbbf24", inner: "#fef3c7", label: "Speaking" },
  searching: { glow: "#06b6d4", accent: "#67e8f9", inner: "#cffafe", label: "Searching" },
  error:     { glow: "#ef4444", accent: "#f87171", inner: "#fee2e2", label: "Error" },
};

// Scale variations per state - no glow for listening
const scaleMap: Record<Phase, number> = {
  idle: 1.0,
  listening: 1.08,
  thinking: 1.03,
  speaking: 1.12,
  searching: 0.98,
  error: 0.92
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
    console.error('SAGE Orb Error:', error, errorInfo);
  }

  render() {
    if (this.state.hasError) {
      return (
        <div className="min-h-screen w-full bg-neutral-950 text-neutral-100 flex items-center justify-center">
          <div className="text-center space-y-4 p-8">
            <div className="text-6xl">�</div>
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

function SageOrbInner(){
  // WS config
  const WS_HOST = (typeof window !== 'undefined' && window.location) ? window.location.hostname : "localhost";
  const WS_URL = `ws://${WS_HOST}:8765/ws/status`;

  const [connected, setConnected] = useState(false);
  const [latency, setLatency] = useState<number|null>(null);
  const [phase, setPhase] = useState<Phase>('idle');
  const [battery, setBattery] = useState({ pct: 0, charging: false });
  const [viseme, setViseme] = useState(0);
  const [drawerOpen, setDrawerOpen] = useState(false);
  const [popups, setPopups] = useState<{id:number; text:string; kind?:'info'|'ok'|'warn'|'error'}[]>([]);
  
  const idRef = useRef(1);
  const popupTimersRef = useRef<Map<number, number>>(new Map());

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

  useEffect(() => {
    return () => {
      popupTimersRef.current.forEach(timer => clearTimeout(timer));
      popupTimersRef.current.clear();
    };
  }, []);

  // WebSocket
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
          
          pingTimer = window.setInterval(() => {
            if (ws && ws.readyState === WebSocket.OPEN) {
              const now = performance.now();
              pingTimestamps.set(now, now);
              ws.send(JSON.stringify({ type: "ping", t: now }));
              
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
              setPhase(m.phase as Phase);
            }
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
          return PHASES[(idx + 1) % PHASES.length];
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

  return (
    <div style={{ 
      minHeight: '100vh', 
      width: '100%', 
      background: '#0a0a0a', 
      color: '#f5f5f5', 
      overflow: 'hidden',
      position: 'relative'
    }}>
      <style>{ORB_CSS}</style>

      {/* HUD */}
      <div style={{ position: 'fixed', top: '12px', right: '16px', display: 'flex', alignItems: 'center', gap: '12px', zIndex: 30, userSelect: 'none' }}>
        <ConnPill ok={connected} latency={latency} />
        <MiniBattery pct={battery.pct} charging={battery.charging} />
      </div>

      {/* Debug nub */}
      <button 
        aria-label="Toggle debug drawer (D)" 
        onClick={() => setDrawerOpen(v => !v)} 
        style={{
          position: 'fixed',
          bottom: '8px',
          right: '8px',
          zIndex: 30,
          width: '16px',
          height: '16px',
          borderRadius: '50%',
          background: 'rgba(64, 64, 64, 0.7)',
          border: '1px solid #525252',
          cursor: 'pointer'
        }}
      />

      {/* Popups */}
      <div style={{ position: 'fixed', top: '64px', right: '16px', zIndex: 30 }} role="alert" aria-live="polite">
        <AnimatePresence>
          {popups.map(p => <Popup key={p.id} text={p.text} kind={p.kind} />)}
        </AnimatePresence>
      </div>

      {/* ORB - Full screen */}
      <div style={{ position: 'fixed', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
        <OrbContainer phase={phase} viseme={viseme} />
      </div>

      <Drawer open={drawerOpen} onClose={() => setDrawerOpen(false)}>
        <div className="p-3 text-sm space-y-2">
          <div><strong>Phase:</strong> {phasePalette[phase].label}</div>
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

function OrbContainer({ phase, viseme }: { phase: Phase; viseme: number }) {
  const pal = phasePalette[phase];

  return (
    <motion.div 
      className="orb-container"
      animate={{ 
        scale: scaleMap[phase],
      }}
      transition={{ 
        type: "spring", 
        stiffness: 120, 
        damping: 20,
        mass: 1.2
      }}
    >
      {/* Outer glow - hide for listening state */}
      {phase !== 'listening' && (
        <motion.div 
          className="orb-glow"
          animate={{ 
            background: `radial-gradient(circle, ${hexA(pal.glow, 0.4)} 0%, ${hexA(pal.glow, 0.1)} 40%, transparent 70%)`
          }}
          transition={{ duration: 0.8 }}
        />
      )}

      {/* Glass sphere shell */}
      <motion.div 
        className="orb-shell"
        animate={{
          boxShadow: `
            inset 0 0 60px ${hexA(pal.accent, 0.15)},
            inset 20px -20px 40px ${hexA(pal.inner, 0.1)},
            0 0 80px ${hexA(pal.glow, 0.3)},
            0 0 120px ${hexA(pal.glow, 0.15)}
          `
        }}
        transition={{ duration: 0.8 }}
      >
        {/* Highlight */}
        <motion.div 
          className="orb-highlight"
          animate={{
            background: `radial-gradient(circle at 30% 30%, ${hexA('#ffffff', 0.4)} 0%, transparent 50%)`
          }}
          transition={{ duration: 0.6 }}
        />

        {/* Contents - state specific */}
        <div className="orb-contents">
          <AnimatePresence mode="wait">
            {phase === 'idle' && <IdleParticles key="idle" />}
            {phase === 'listening' && <ListeningRipples key="listening" />}
            {phase === 'thinking' && <ThinkingNetwork key="thinking" />}
            {phase === 'speaking' && <SpeakingWave key="speaking" viseme={viseme} />}
            {phase === 'searching' && <SearchingOrbit key="searching" />}
            {phase === 'error' && <ErrorChaos key="error" />}
          </AnimatePresence>
        </div>
      </motion.div>
    </motion.div>
  );
}

// STATE CONTENTS

function IdleParticles() {
  const particles = useMemo(() => 
    Array.from({ length: 40 }, (_, i) => ({
      id: i,
      x: (Math.random() - 0.5) * 180,
      y: (Math.random() - 0.5) * 180,
      size: 1 + Math.random() * 2,
      duration: 3 + Math.random() * 4,
      delay: Math.random() * 3
    }))
  , []);

  return (
    <motion.div 
      className="state-content"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.6 }}
    >
      {particles.map(p => (
        <motion.div
          key={p.id}
          className="particle idle-particle"
          style={{
            left: '50%',
            top: '50%',
            width: p.size,
            height: p.size,
          }}
          animate={{
            x: [p.x, p.x + (Math.random() - 0.5) * 40],
            y: [p.y, p.y + (Math.random() - 0.5) * 40],
            opacity: [0.3, 0.7, 0.3]
          }}
          transition={{
            duration: p.duration,
            repeat: Infinity,
            delay: p.delay,
            ease: "easeInOut"
          }}
        />
      ))}
    </motion.div>
  );
}

function ListeningRipples() {
  const [rotation, setRotation] = useState(0);
  const rafRef = useRef<number>(0);

  useEffect(() => {
    let last = 0;
    const step = (ts: number) => {
      if (!last) last = ts;
      const dt = Math.min(0.032, (ts - last) / 1000);
      last = ts;
      
      setRotation(r => r + dt * 60);
      
      rafRef.current = requestAnimationFrame(step);
    };
    
    rafRef.current = requestAnimationFrame(step);
    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, []);

  // Create orbital rings at different angles
  const rings = useMemo(() => [
    { radius: 70, tilt: 0, speed: 1 },
    { radius: 90, tilt: 60, speed: -0.7 },
    { radius: 110, tilt: 120, speed: 0.5 },
  ], []);

  // Energy particles on the rings
  const particles = useMemo(() => 
    Array.from({ length: 24 }, (_, i) => ({
      id: i,
      ringIndex: i % 3,
      offset: (i * 137.5) % 360,
      size: 2 + (i % 3),
    }))
  , []);

  return (
    <motion.div 
      className="state-content listening-field"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.6 }}
    >
      {/* Orbital rings */}
      {rings.map((ring, idx) => (
        <div
          key={idx}
          className="orbit-ring"
          style={{
            width: ring.radius * 2,
            height: ring.radius * 2,
            transform: `rotateX(${ring.tilt}deg) rotateY(${rotation * ring.speed}deg)`,
          }}
        />
      ))}

      {/* Energy particles orbiting */}
      {particles.map(p => {
        const ring = rings[p.ringIndex];
        const angle = ((rotation * ring.speed + p.offset) * Math.PI) / 180;
        const x = Math.cos(angle) * ring.radius;
        const y = Math.sin(angle) * ring.radius * Math.cos((ring.tilt * Math.PI) / 180);
        const z = Math.sin(angle) * ring.radius * Math.sin((ring.tilt * Math.PI) / 180);
        
        return (
          <div
            key={p.id}
            className="energy-particle"
            style={{
              left: '50%',
              top: '50%',
              transform: `translate3d(${x}px, ${y}px, ${z}px)`,
              width: p.size,
              height: p.size,
            }}
          />
        );
      })}

      {/* Central energy core */}
      <motion.div 
        className="listening-core"
        animate={{ 
          scale: [1, 1.2, 1],
          opacity: [0.8, 1, 0.8]
        }}
        transition={{
          duration: 1.5,
          repeat: Infinity,
          ease: "easeInOut"
        }}
      />

      {/* Electric arcs */}
      <svg className="arc-svg" viewBox="-150 -150 300 300">
        <defs>
          <linearGradient id="arcGrad" x1="0%" y1="0%" x2="100%" y2="0%">
            <stop offset="0%" stopColor="rgba(52, 211, 153, 0)" />
            <stop offset="50%" stopColor="rgba(52, 211, 153, 0.8)" />
            <stop offset="100%" stopColor="rgba(52, 211, 153, 0)" />
          </linearGradient>
        </defs>
        <motion.path
          d="M -80,0 Q -40,-30 0,0 Q 40,30 80,0"
          stroke="url(#arcGrad)"
          strokeWidth="2"
          fill="none"
          animate={{
            d: [
              "M -80,0 Q -40,-30 0,0 Q 40,30 80,0",
              "M -80,0 Q -40,30 0,0 Q 40,-30 80,0",
              "M -80,0 Q -40,-30 0,0 Q 40,30 80,0",
            ],
            opacity: [0.6, 0.9, 0.6]
          }}
          transition={{
            duration: 2,
            repeat: Infinity,
            ease: "easeInOut"
          }}
        />
      </svg>
    </motion.div>
  );
}

function ThinkingNetwork() {
  const [rotation, setRotation] = useState(0);
  const [pulsePhase, setPulsePhase] = useState(0);
  const rafRef = useRef<number>(0);

  useEffect(() => {
    let last = 0;
    const step = (ts: number) => {
      if (!last) last = ts;
      const dt = Math.min(0.032, (ts - last) / 1000);
      last = ts;
      
      setRotation(r => r + dt * 15);
      setPulsePhase(p => p + dt * 2);
      
      rafRef.current = requestAnimationFrame(step);
    };
    
    rafRef.current = requestAnimationFrame(step);
    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, []);

  // Create a 3D helix/DNA-like structure
  const helixPoints = useMemo(() => {
    const points = [];
    for (let i = 0; i < 40; i++) {
      const t = (i / 40) * Math.PI * 4;
      points.push({
        id: i,
        angle: t,
        radius: 60,
        y: (i - 20) * 6,
      });
    }
    return points;
  }, []);

  return (
    <motion.div 
      className="state-content"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.5 }}
    >
      <div className="helix-container" style={{ transform: `rotateY(${rotation}deg)` }}>
        {helixPoints.map((p, i) => {
          const x = Math.cos(p.angle) * p.radius;
          const z = Math.sin(p.angle) * p.radius;
          const pulse = Math.sin(pulsePhase + i * 0.3) * 0.5 + 0.5;
          const opacity = 0.4 + pulse * 0.6;
          const scale = 0.7 + pulse * 0.5;
          
          return (
            <div
              key={p.id}
              className="helix-particle"
              style={{
                transform: `translate3d(${x}px, ${p.y}px, ${z}px) scale(${scale})`,
                opacity: opacity,
              }}
            />
          );
        })}
      </div>
      
      {/* Energy core */}
      <motion.div 
        className="thinking-core"
        animate={{ 
          scale: [1, 1.15, 1],
          opacity: [0.6, 0.9, 0.6]
        }}
        transition={{
          duration: 2,
          repeat: Infinity,
          ease: "easeInOut"
        }}
      />
    </motion.div>
  );
}

function SpeakingWave({ viseme }: { viseme: number }) {
  const bars = 32;
  const [t, setT] = useState(0);
  const rafRef = useRef<number>(0);
  
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

  return (
    <motion.div 
      className="state-content"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.4 }}
    >
      <div className="wave-container">
        {Array.from({ length: bars }, (_, i) => {
          const angle = (i / bars) * Math.PI * 2;
          const base = 0.3 + 0.7 * Math.sin(t * 3 + i * 0.4);
          const focus = Math.max(0, 1 - Math.abs(((viseme % bars) - i)) / 6);
          const height = 15 + 35 * (base * 0.6 + focus * 0.4);
          
          return (
            <div
              key={i}
              className="wave-bar"
              style={{
                transform: `rotate(${angle}rad) translateY(-85px)`,
                height: `${height}px`,
                opacity: 0.5 + focus * 0.5
              }}
            />
          );
        })}
      </div>
    </motion.div>
  );
}

function SearchingOrbit() {
  const [angle, setAngle] = useState(0);
  
  useEffect(() => {
    const timer = setInterval(() => {
      setAngle(a => (a + 2) % 360);
    }, 30);
    return () => clearInterval(timer);
  }, []);

  const particles = useMemo(() => 
    Array.from({ length: 20 }, (_, i) => ({
      id: i,
      radius: 40 + (i % 3) * 25,
      offset: (i * 137.5) % 360,
      speed: 1 + (i % 3) * 0.5,
      size: 2 + (i % 3)
    }))
  , []);

  return (
    <motion.div 
      className="state-content"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.5 }}
    >
      {particles.map(p => {
        const a = ((angle * p.speed + p.offset) * Math.PI) / 180;
        const x = Math.cos(a) * p.radius;
        const y = Math.sin(a) * p.radius;
        
        return (
          <div
            key={p.id}
            className="particle orbit-particle"
            style={{
              left: '50%',
              top: '50%',
              transform: `translate(${x}px, ${y}px)`,
              width: p.size,
              height: p.size,
            }}
          />
        );
      })}
      <div className="orbit-center" />
    </motion.div>
  );
}

function ErrorChaos() {
  const particles = useMemo(() => 
    Array.from({ length: 50 }, (_, i) => ({
      id: i,
      x: (Math.random() - 0.5) * 200,
      y: (Math.random() - 0.5) * 200,
      size: 1 + Math.random() * 3,
      duration: 0.3 + Math.random() * 0.5
    }))
  , []);

  return (
    <motion.div 
      className="state-content"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.3 }}
    >
      {particles.map(p => (
        <motion.div
          key={p.id}
          className="particle error-particle"
          style={{
            left: '50%',
            top: '50%',
            width: p.size,
            height: p.size,
          }}
          animate={{
            x: [0, p.x, (Math.random() - 0.5) * 200],
            y: [0, p.y, (Math.random() - 0.5) * 200],
            opacity: [0.8, 0.3, 0.8]
          }}
          transition={{
            duration: p.duration,
            repeat: Infinity,
            ease: "easeInOut"
          }}
        />
      ))}
      <motion.div 
        className="error-core"
        animate={{ 
          scale: [1, 1.2, 1],
          opacity: [0.8, 0.4, 0.8]
        }}
        transition={{
          duration: 0.8,
          repeat: Infinity,
          ease: "easeInOut"
        }}
      />
    </motion.div>
  );
}

// HUD Components
function ConnPill({ ok, latency }: { ok: boolean; latency: number | null }) {
  return (
    <div className="hud-pill" role="status" aria-live="polite">
      <span className={`dot ${ok ? 'ok' : 'bad'}`} />
      <span>
        {ok ? 'Connected' : 'Disconnected'}
        {ok && latency !== null ? ` " ${latency}ms` : ''}
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
  useEffect(() => {
    if (!open) return;
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    };
    window.addEventListener('keydown', handleEscape);
    return () => window.removeEventListener('keydown', handleEscape);
  }, [open, onClose]);
  
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
export default function SageOrb() {
  return (
    <ErrorBoundary>
      <SageOrbInner />
    </ErrorBoundary>
  );
}

// CSS
const ORB_CSS = `
:root { --bg: #0a0a0a; }
html, body { background: var(--bg); margin: 0; padding: 0; }

/* Orb Container */
.orb-container {
  position: relative;
  width: 320px;
  height: 320px;
  will-change: transform;
}

.orb-glow {
  position: absolute;
  inset: -60px;
  border-radius: 50%;
  filter: blur(40px);
  pointer-events: none;
  z-index: 1;
}

.orb-shell {
  position: absolute;
  inset: 0;
  border-radius: 50%;
  background: linear-gradient(135deg, rgba(255,255,255,0.05) 0%, rgba(0,0,0,0.3) 100%);
  backdrop-filter: blur(1px);
  overflow: hidden;
  z-index: 2;
}

.orb-highlight {
  position: absolute;
  inset: 0;
  border-radius: 50%;
  pointer-events: none;
  z-index: 3;
}

.orb-contents {
  position: absolute;
  inset: 0;
  border-radius: 50%;
  overflow: hidden;
  z-index: 2;
}

.state-content {
  position: absolute;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
}

/* Particles */
.particle {
  position: absolute;
  border-radius: 50%;
  pointer-events: none;
}

.idle-particle {
  background: radial-gradient(circle, rgba(148, 163, 184, 0.8) 0%, transparent 70%);
  box-shadow: 0 0 8px rgba(148, 163, 184, 0.6);
}

.orbit-particle {
  background: radial-gradient(circle, rgba(103, 232, 249, 0.9) 0%, transparent 70%);
  box-shadow: 0 0 10px rgba(103, 232, 249, 0.7);
}

.error-particle {
  background: radial-gradient(circle, rgba(248, 113, 113, 0.9) 0%, transparent 70%);
  box-shadow: 0 0 8px rgba(248, 113, 113, 0.6);
}

/* Listening - Electromagnetic Field */
.listening-field {
  perspective: 800px;
  transform-style: preserve-3d;
}

.orbit-ring {
  position: absolute;
  top: 50%;
  left: 50%;
  border: 1.5px solid rgba(52, 211, 153, 0.3);
  border-radius: 50%;
  transform-style: preserve-3d;
  pointer-events: none;
  box-shadow: 0 0 15px rgba(52, 211, 153, 0.2), inset 0 0 15px rgba(52, 211, 153, 0.1);
}

.energy-particle {
  position: absolute;
  background: radial-gradient(circle, rgba(52, 211, 153, 1) 0%, rgba(16, 185, 129, 0.6) 100%);
  border-radius: 50%;
  box-shadow: 0 0 10px rgba(52, 211, 153, 0.9);
  pointer-events: none;
}

.listening-core {
  position: absolute;
  top: 50%;
  left: 50%;
  width: 16px;
  height: 16px;
  margin: -8px 0 0 -8px;
  background: radial-gradient(circle, rgba(52, 211, 153, 1) 0%, rgba(16, 185, 129, 0.5) 100%);
  border-radius: 50%;
  box-shadow: 0 0 25px rgba(52, 211, 153, 1), 0 0 50px rgba(16, 185, 129, 0.5);
  z-index: 10;
}

.arc-svg {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

/* Thinking - Helix */
.helix-container {
  position: relative;
  width: 100%;
  height: 100%;
  transform-style: preserve-3d;
  perspective: 1000px;
}

.helix-particle {
  position: absolute;
  top: 50%;
  left: 50%;
  width: 6px;
  height: 6px;
  margin: -3px 0 0 -3px;
  background: radial-gradient(circle, rgba(165, 180, 252, 1) 0%, rgba(129, 140, 248, 0.6) 100%);
  border-radius: 50%;
  box-shadow: 0 0 12px rgba(165, 180, 252, 0.8);
  pointer-events: none;
}

.thinking-core {
  position: absolute;
  top: 50%;
  left: 50%;
  width: 30px;
  height: 30px;
  margin: -15px 0 0 -15px;
  background: radial-gradient(circle, rgba(165, 180, 252, 0.9) 0%, rgba(99, 102, 241, 0.4) 100%);
  border-radius: 50%;
  box-shadow: 0 0 30px rgba(165, 180, 252, 0.8), 0 0 60px rgba(99, 102, 241, 0.4);
}

/* Speaking Wave */
.wave-container {
  position: relative;
  width: 100%;
  height: 100%;
}

.wave-bar {
  position: absolute;
  top: 50%;
  left: 50%;
  width: 4px;
  background: linear-gradient(to top, rgba(251, 191, 36, 0.8), rgba(245, 158, 11, 0.9));
  border-radius: 2px;
  transform-origin: bottom center;
  transition: height 0.15s ease-out, opacity 0.15s ease-out;
  box-shadow: 0 0 8px rgba(251, 191, 36, 0.6);
}

/* Searching Orbit */
.orbit-center {
  position: absolute;
  top: 50%;
  left: 50%;
  width: 12px;
  height: 12px;
  margin: -6px 0 0 -6px;
  background: radial-gradient(circle, rgba(103, 232, 249, 1) 0%, rgba(6, 182, 212, 0.6) 100%);
  border-radius: 50%;
  box-shadow: 0 0 20px rgba(103, 232, 249, 0.9);
}

/* Error Chaos */
.error-core {
  position: absolute;
  top: 50%;
  left: 50%;
  width: 40px;
  height: 40px;
  margin: -20px 0 0 -20px;
  background: radial-gradient(circle, rgba(239, 68, 68, 0.8) 0%, transparent 70%);
  border-radius: 50%;
  box-shadow: 0 0 30px rgba(239, 68, 68, 0.8);
}

/* HUD */
.hud-pill {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 6px 12px;
  background: rgba(38, 38, 38, 0.85);
  backdrop-filter: blur(8px);
  border: 1px solid rgba(82, 82, 82, 0.3);
  border-radius: 20px;
  font-size: 13px;
  color: #d4d4d4;
}

.dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
}

.dot.ok {
  background: #22c55e;
  box-shadow: 0 0 8px #22c55e;
}

.dot.bad {
  background: #ef4444;
  box-shadow: 0 0 8px #ef4444;
}

/* Popup */
.popup {
  padding: 10px 16px;
  background: rgba(38, 38, 38, 0.95);
  backdrop-filter: blur(12px);
  border: 1px solid var(--c);
  border-radius: 8px;
  font-size: 13px;
  color: #e5e5e5;
  box-shadow: 0 0 20px rgba(0, 0, 0, 0.5), 0 0 10px var(--c);
  max-width: 280px;
}

/* Drawer */
.drawer {
  position: fixed;
  inset: 0;
  z-index: 40;
  pointer-events: none;
}

.drawer.open {
  pointer-events: auto;
}

.drawer-backdrop {
  position: absolute;
  inset: 0;
  background: rgba(0, 0, 0, 0.6);
  opacity: 0;
  transition: opacity 0.2s;
}

.drawer.open .drawer-backdrop {
  opacity: 1;
}

.drawer-body {
  position: absolute;
  bottom: 0;
  left: 0;
  right: 0;
  max-height: 50vh;
  background: rgba(23, 23, 23, 0.98);
  backdrop-filter: blur(12px);
  border-top: 1px solid rgba(82, 82, 82, 0.3);
  transform: translateY(100%);
  transition: transform 0.3s ease-out;
  overflow-y: auto;
}

.drawer.open .drawer-body {
  transform: translateY(0);
}

.kbd {
  display: inline-block;
  padding: 2px 6px;
  font-family: ui-monospace, monospace;
  font-size: 11px;
  background: rgba(64, 64, 64, 0.5);
  border: 1px solid rgba(115, 115, 115, 0.4);
  border-radius: 4px;
  color: #d4d4d4;
}
`;
// Mount
const root = document.getElementById("root");
if (root) {
  ReactDOM.createRoot(root).render(<SageOrb />);
}