#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Triggered Motor Logger GUI (Tkinter + pyX2Cscope) — FIXED f=50, Correct channel mapping & scaling
------------------------------------------------------------------------------------------------
• TRIGGER SOURCE: OmegaElectrical (actual speed), rising edge.
• Trigger fixed at raw level 700 with 10% delay.
• Sample-time factor is FIXED to f=50 (short, high-resolution window).
  - Base raw period 50 µs → Ts = 2.5 ms (≈ 400 Hz). Expected total window ≈ 1225 ms.

Motor control (unchanged):
• One-shot RUN at start, one-shot STOP after 10 seconds (or early STOP).
• Writes velocityReference once using Speed (RPM) and Scale (RPM per count).

Channels:
• idqCmd_q  → motor.idqCmd.q
• Idq_q     → motor.idq.q
• Idq_d     → motor.idq.d
• OmegaElectrical → motor.omegaElectrical
• OmegaCmd  → motor.omegaCmd
• Per-channel scaling entries (default: currents=0.0003125, speeds=0.19913 RPM/raw).
• “Lock Scales” button freezes scaling before a run.

Export:
• “Export…” saves a TSV with **scaled** values (real units). No auto-save.

FIXES in this version:
• **Correct channel mapping:** data are matched by the **exact variable symbol key** returned by the scope,
  not by dictionary order, eliminating the issue where `idqCmd_q` and `Idq_q` appeared identical.
• **Correct scaling in export:** TSV writes **scaled** series (real = raw × scale) per channel label.
"""

from __future__ import annotations

import csv
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Tkinter GUI
import tkinter as tk
from tkinter import filedialog, messagebox

# Optional matplotlib for plots
try:
    import matplotlib.pyplot as plt
    HAS_PLT = True
except Exception:
    plt = None
    HAS_PLT = False

# Serial only for listing ports
import serial.tools.list_ports  # type: ignore

# Primary dependency — prefer package root import to avoid circular imports
try:
    from pyx2cscope import X2CScope  # type: ignore
except Exception:  # last-resort fallback if package doesn’t re-export
    import importlib
    _mod = importlib.import_module("pyx2cscope.x2cscope")
    X2CScope = getattr(_mod, "X2CScope")


# ----------------------------- Fixed sampling + timings -----------------------------

RAW_SAMPLE_TIME_US = 50.0      # fixed raw base period per sample (µs)
FACTOR_F = 50                  # FIXED decimation factor for this app
TS_S = (FACTOR_F * RAW_SAMPLE_TIME_US) / 1_000_000.0    # 0.0025 s
TS_MS = TS_S * 1_000.0                                  # 2.5 ms per sample
FS_HZ = 1.0 / TS_S if TS_S > 0 else 0.0                 # 400 Hz

# Project expectation for f=50
TOTAL_MS_EXPECTED = 1225.0

# Motor run/stop behavior
RUN_SECONDS = 10.0
SLEEP_POLL = 0.05


# ----------------------------- Variables -----------------------------

# Monitor variables (label, ELF symbol)
MONITOR_VARS: List[Tuple[str, str]] = [
    ("idqCmd_q",        "motor.idqCmd.q"),
    ("Idq_q",           "motor.idq.q"),
    ("Idq_d",           "motor.idq.d"),
    ("OmegaElectrical", "motor.omegaElectrical"),
    ("OmegaCmd",        "motor.omegaCmd"),
]

DEFAULT_SCALES: Dict[str, float] = {
    "idqCmd_q": 0.0003125,
    "Idq_q": 0.0003125,
    "Idq_d": 0.0003125,
    "OmegaElectrical": 0.19913,
    "OmegaCmd": 0.19913,
}

# Control variables (ELF symbol)
CTRL_HW_UI      = "app.hardwareUiEnabled"
CTRL_VEL_REF    = "motor.apiData.velocityReference"  # counts
CTRL_RUN_REQ    = "motor.apiData.runMotorRequest"
CTRL_STOP_REQ   = "motor.apiData.stopMotorRequest"


# ----------------------------- Local TriggerConfig (duck-typed) -----------------------------

@dataclass
class TriggerConfig:
    variable: object       # Variable handle
    trigger_level: int = 0
    trigger_mode: int = 1  # 0 Auto, 1 Triggered (use 1)
    trigger_delay: int = 10  # percent of scope size pre/post split
    trigger_edge: int = 0  # 0 Rising, 1 Falling


# ----------------------------- Utilities -----------------------------

def list_ports() -> List[str]:
    return [p.device for p in serial.tools.list_ports.comports()]

def safe_float(s: str, default: float = 0.0) -> float:
    try:
        return float(s)
    except Exception:
        return default

def try_write(scope: X2CScope, handle, value) -> bool:
    """Write value to a variable handle using whichever API is available."""
    try:
        if hasattr(handle, "set_value"):
            handle.set_value(value)  # type: ignore
            return True
    except Exception:
        pass
    try:
        if hasattr(scope, "write"):
            scope.write(handle, value)  # type: ignore
            return True
    except Exception:
        pass
    return False

def try_clear_channels(scope: X2CScope) -> None:
    for name in ("clear_all_scope_channel", "clear_scope_channels", "clear_scope_channel"):
        if hasattr(scope, name):
            try:
                getattr(scope, name)()  # type: ignore
                return
            except Exception:
                pass

def get_total_ms_via_api(scope: X2CScope, time_us: float) -> float:
    """
    Uses:
        get_scope_sample_time(self, time: float) -> float
    Returns total real duration (ms) of a dataset for the given raw time.
    """
    if hasattr(scope, "get_scope_sample_time"):
        try:
            return float(scope.get_scope_sample_time(time_us))  # type: ignore
        except TypeError:
            try:
                ret = scope.get_scope_sample_time()  # type: ignore
                if isinstance(ret, (int, float)):
                    return float(ret)
            except Exception:
                pass
        except Exception:
            pass
    return 0.0


# ----------------------------- Capture Worker -----------------------------

class CaptureWorker(threading.Thread):
    """
    • One-shot RUN immediately (sets velocityReference first).
    • Configure scope channels; set sample_time factor = 50 (fixed).
    • Configure TRIGGER: OmegaElectrical rising edge, fixed raw level 700 with 10% delay.
    • request_scope_data() and wait until is_scope_data_ready() — read the triggered dataset once.
    • Keep motor running until 10 s; then send one-shot STOP (unless early stop requested).
    • No auto-save — Export button writes TSV later.
    """
    def __init__(
        self,
        app_ref,                     # MotorLoggerApp
        x2c: X2CScope,
        handles: Dict[str, object],  # symbol -> handle
        variables: List[Tuple[str, str]],  # (label, symbol)
        counts: float,               # velocityReference to write (counts)
        trigger_raw_level: float,    # RAW units level for trigger
        trigger_delay_pct: int,      # % delay
        scales: Dict[str, float],    # per-label scaling for plotting/export
    ):
        super().__init__(daemon=True)
        self.app = app_ref
        self.x2c = x2c
        self.handles = handles
        self.variables = variables
        self.counts = counts
        self.trigger_raw_level = int(trigger_raw_level)
        self.trigger_delay_pct = max(0, min(90, int(trigger_delay_pct)))
        self.scales = scales

        self._stop_flag = threading.Event()
        self._run_sent = False
        self._stop_sent = False

        # Results (raw)
        self.data_raw: Dict[str, List[float]] = {lbl: [] for (lbl, _sym) in variables}
        self.t_axis: List[float] = []
        self.total_ms_reported: float = 0.0

    def stop_early(self):
        if not self._stop_sent:
            h = self.handles.get(CTRL_STOP_REQ)
            if h is not None:
                try_write(self.x2c, h, 1)
            self._stop_sent = True
        self._stop_flag.set()

    def _ui_status(self, text: str):
        self.app.safe_set_status(text)

    def run(self):
        try:
            # Configure scope channels
            try_clear_channels(self.x2c)
            for (_lbl, sym) in self.variables:
                h = self.handles.get(sym)
                if h is not None:
                    try:
                        self.x2c.add_scope_channel(h)  # type: ignore
                    except Exception:
                        pass

            # Set fixed decimation factor f = 50
            try:
                self.x2c.set_sample_time(FACTOR_F)  # type: ignore
            except Exception:
                pass

            # Configure trigger using OmegaElectrical
            omega_elec_h = self.handles.get("motor.omegaElectrical")
            if omega_elec_h is None:
                # Some builds may have different keying, but we always added by this symbol
                omega_elec_h = self.handles.get("OmegaElectrical")
            if omega_elec_h is None:
                self._ui_status("Warning: OmegaElectrical handle not resolved; proceeding without trigger.")
                if hasattr(self.x2c, "reset_scope_trigger"):
                    try:
                        self.x2c.reset_scope_trigger()  # type: ignore
                    except Exception:
                        pass
            else:
                cfg = TriggerConfig(
                    variable=omega_elec_h,
                    trigger_level=self.trigger_raw_level,
                    trigger_mode=1,                # Triggered
                    trigger_delay=self.trigger_delay_pct,
                    trigger_edge=0,                # Rising
                )
                if hasattr(self.x2c, "set_scope_trigger"):
                    try:
                        self.x2c.set_scope_trigger(cfg)  # type: ignore
                        self._ui_status(
                            f"Trigger set: OmegaElectrical rising @ raw={self.trigger_raw_level}, "
                            f"delay={self.trigger_delay_pct}%."
                        )
                    except Exception as e:
                        self._ui_status(f"Failed to set trigger (continuing): {e}")

            # Write velocityReference once
            vref = self.handles.get(CTRL_VEL_REF)
            if vref is not None:
                try_write(self.x2c, vref, int(self.counts))

            # One-shot RUN now
            run_h = self.handles.get(CTRL_RUN_REQ)
            if run_h is not None:
                try_write(self.x2c, run_h, 1)
                self._run_sent = True
                self._ui_status("RUN sent (one-shot). Waiting for triggered dataset…")

            # Start a timer to STOP at 10 s (unless early stop)
            t_run_start = time.time()

            # Request scope data ONCE (triggered capture)
            try:
                self.x2c.request_scope_data()  # type: ignore
            except Exception:
                pass

            # Wait for dataset to be ready (triggered) with a reasonable timeout
            t0 = time.time()
            timeout_s = 6.0
            while not self._stop_flag.is_set():
                ready = False
                try:
                    ready = bool(self.x2c.is_scope_data_ready())  # type: ignore
                except Exception:
                    pass

                if ready:
                    break

                if (time.time() - t0) > timeout_s:
                    self._ui_status("Timeout waiting for triggered dataset. (No trigger?)")
                    break

                # Also stop the motor if overall 10 s elapsed
                if (time.time() - t_run_start) >= RUN_SECONDS:
                    break

                time.sleep(SLEEP_POLL)

            # Read the single dataset (valid, trigger-filtered)
            try:
                channels = self.x2c.get_scope_channel_data(valid_data=True)  # type: ignore
            except Exception:
                channels = {}

            # Store aligned channel data (RAW) — **EXACT KEY MAPPING**
            if isinstance(channels, dict) and channels:
                # Expect keys to be the exact symbol names used when adding channels.
                # Build strictly by exact lookup to avoid cross-channel duplication.
                n_min_candidates: List[int] = []
                per_symbol_series: Dict[str, List[float]] = {}
                for _lbl, sym in self.variables:
                    series = channels.get(sym)
                    if series is not None:
                        per_symbol_series[sym] = list(series)
                        n_min_candidates.append(len(series))
                    else:
                        per_symbol_series[sym] = []

                n_min = min(n_min_candidates) if n_min_candidates else 0

                if n_min > 0:
                    for lbl, sym in self.variables:
                        self.data_raw[lbl] = per_symbol_series.get(sym, [])[:n_min]

                    # Build time axis by fixed Ts = 2.5 ms
                    self.t_axis = [i * TS_S for i in range(n_min)]
                    self._ui_status(
                        f"Captured triggered dataset: {n_min} samples/channel. "
                        f"Ts={TS_MS:.3f} ms (Fs≈{FS_HZ:.1f} Hz)."
                    )
                else:
                    self._ui_status("Triggered dataset empty or incomplete (n_min = 0).")
            else:
                self._ui_status("No scope data returned (channels empty).")

            # Query device for total ms (for info only)
            try:
                self.total_ms_reported = get_total_ms_via_api(self.x2c, RAW_SAMPLE_TIME_US)
            except Exception:
                self.total_ms_reported = 0.0
            if self.total_ms_reported <= 0.0:
                self.total_ms_reported = TOTAL_MS_EXPECTED

            # Keep motor running until 10 s total, then STOP once
            while not self._stop_flag.is_set() and (time.time() - t_run_start) < RUN_SECONDS:
                time.sleep(SLEEP_POLL)

            if not self._stop_sent:
                stop_h = self.handles.get(CTRL_STOP_REQ)
                if stop_h is not None:
                    try_write(self.x2c, stop_h, 1)
                self._stop_sent = True
                self._ui_status("STOP sent (one-shot).")

            # Notify GUI with RAW data; GUI will scale for plots/export
            self.app.on_capture_complete(self.data_raw, self.t_axis, self.total_ms_reported, self.scales)
        except Exception as e:
            self._ui_status("Error during capture.")
            self.app.safe_show_error("Capture error", f"{e}\n\n{traceback.format_exc()}")
        finally:
            # Safety net: if RUN was sent but we never STOPped, send STOP once
            if self._run_sent and not self._stop_sent:
                try:
                    h = self.handles.get(CTRL_STOP_REQ)
                    if h is not None:
                        try_write(self.x2c, h, 1)
                except Exception:
                    pass


# ----------------------------- Main App -----------------------------

class MotorLoggerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Triggered Motor Logger (Tk + pyX2Cscope) — f=50 ⇒ Ts=2.5 ms, Trigger: OmegaElectrical")
        self.geometry("1180x780")

        # Session state
        self.x2c: Optional[X2CScope] = None
        self.connected = False
        self.elf_path: Optional[Path] = None
        self.port: Optional[str] = None
        self.baud: int = 115200
        self.handles: Dict[str, object] = {}

        # Last capture (RAW and Scaled)
        self.last_raw: Optional[Dict[str, List[float]]] = None
        self.last_scaled: Optional[Dict[str, List[float]]] = None
        self.last_t: Optional[List[float]] = None
        self.last_total_ms: float = 0.0
        self.last_scales: Dict[str, float] = {}

        self.scales_locked: bool = False

        self.worker: Optional[CaptureWorker] = None

        self._build_ui()
        self._refresh_ports()

    # ---------- UI ----------

    def _build_ui(self):
        pad = {"padx": 8, "pady": 6}

        # Row 0: ELF + Port + Baud
        row0 = tk.Frame(self)
        row0.pack(fill="x", **pad)

        tk.Label(row0, text="ELF:").pack(side="left")
        self.lbl_elf = tk.Label(row0, text="(none)", width=44, anchor="w")
        self.lbl_elf.pack(side="left", padx=6)
        tk.Button(row0, text="Browse…", command=self.on_browse_elf).pack(side="left", padx=4)

        tk.Label(row0, text="Port:").pack(side="left", padx=(18, 0))
        self.cbo_port = tk.StringVar(self)
        self.ddl_port = tk.OptionMenu(row0, self.cbo_port, "")
        self.ddl_port.config(width=16)
        self.ddl_port.pack(side="left")
        tk.Button(row0, text="Refresh", command=self._refresh_ports).pack(side="left", padx=4)

        tk.Label(row0, text="Baud:").pack(side="left", padx=(18, 0))
        self.cbo_baud = tk.StringVar(self, "115200")
        self.ddl_baud = tk.OptionMenu(row0, self.cbo_baud, "115200", "230400", "460800", "921600")
        self.ddl_baud.config(width=10)
        self.ddl_baud.pack(side="left")

        # Row 1: Speed/Scale
        row1 = tk.Frame(self)
        row1.pack(fill="x", **pad)

        tk.Label(row1, text="Speed (RPM):").pack(side="left")
        self.ent_speed = tk.Entry(row1, width=10)
        self.ent_speed.insert(0, "1000")
        self.ent_speed.pack(side="left", padx=6)

        tk.Label(row1, text="Scale (RPM/count):").pack(side="left", padx=(18, 0))
        self.ent_vel_scale = tk.Entry(row1, width=10)
        self.ent_vel_scale.insert(0, str(DEFAULT_SCALES["OmegaElectrical"]))
        self.ent_vel_scale.pack(side="left", padx=6)

        # Row 2: Sampling info + Test button
        row2 = tk.Frame(self)
        row2.pack(fill="x", **pad)

        info = (
            f"Triggered scope: f=50, raw=50 µs → Ts = {TS_MS:.3f} ms per sample (Fs ≈ {FS_HZ:.1f} Hz).  "
            f"Expected total window ≈ {TOTAL_MS_EXPECTED:.0f} ms. Trigger source: OmegaElectrical (rising)."
        )
        self.lbl_info = tk.Label(row2, text=info, anchor="w")
        self.lbl_info.pack(side="left", fill="x", expand=True)

        self.btn_test = tk.Button(row2, text="Test Sampling (get_scope_sample_time 50µs)", command=self.on_test_sampling)
        self.btn_test.pack(side="right")

        # Row 3: Buttons
        row3 = tk.Frame(self)
        row3.pack(fill="x", **pad)

        self.btn_connect = tk.Button(row3, text="Connect", width=12, command=self.on_connect_toggle)
        self.btn_connect.pack(side="left")

        self.btn_start = tk.Button(row3, text="START ▶ (10 s)", width=16, command=self.on_start)
        self.btn_start.pack(side="left", padx=6)

        self.btn_stop = tk.Button(row3, text="STOP ■", width=10, command=self.on_stop)
        self.btn_stop.pack(side="left", padx=6)

        self.btn_export = tk.Button(row3, text="Export…", width=12, command=self.on_export)
        self.btn_export.pack(side="left", padx=(18, 6))

        self.btn_plot_curr = tk.Button(row3, text="Plot Currents", width=14, command=lambda: self.show_plot("currents"))
        self.btn_plot_curr.pack(side="left", padx=6)
        self.btn_plot_omega = tk.Button(row3, text="Plot Omega", width=14, command=lambda: self.show_plot("omega"))
        self.btn_plot_omega.pack(side="left", padx=6)

        # Row 4: Per-channel scaling table + lock button
        row4 = tk.LabelFrame(self, text="Channel scaling (real = raw × scale)")
        row4.pack(fill="x", **pad)
        tk.Label(row4, text="idqCmd_q scale:").grid(row=0, column=0, sticky="e", padx=4, pady=3)
        tk.Label(row4, text="Idq_q scale:").grid(row=0, column=2, sticky="e", padx=4, pady=3)
        tk.Label(row4, text="Idq_d scale:").grid(row=0, column=4, sticky="e", padx=4, pady=3)
        tk.Label(row4, text="OmegaElectrical scale (RPM/raw):").grid(row=1, column=0, sticky="e", padx=4, pady=3)
        tk.Label(row4, text="OmegaCmd scale (RPM/raw):").grid(row=1, column=2, sticky="e", padx=4, pady=3)

        self.ent_scale_idqcmd = tk.Entry(row4, width=10); self.ent_scale_idqcmd.insert(0, str(DEFAULT_SCALES["idqCmd_q"]))
        self.ent_scale_idqq   = tk.Entry(row4, width=10); self.ent_scale_idqq.insert(0, str(DEFAULT_SCALES["Idq_q"]))
        self.ent_scale_idqd   = tk.Entry(row4, width=10); self.ent_scale_idqd.insert(0, str(DEFAULT_SCALES["Idq_d"]))
        self.ent_scale_omegae = tk.Entry(row4, width=12); self.ent_scale_omegae.insert(0, str(DEFAULT_SCALES["OmegaElectrical"]))  # RPM per raw
        self.ent_scale_omegac = tk.Entry(row4, width=10); self.ent_scale_omegac.insert(0, str(DEFAULT_SCALES["OmegaCmd"]))

        self.ent_scale_idqcmd.grid(row=0, column=1, padx=6, pady=3, sticky="w")
        self.ent_scale_idqq.grid(row=0, column=3, padx=6, pady=3, sticky="w")
        self.ent_scale_idqd.grid(row=0, column=5, padx=6, pady=3, sticky="w")
        self.ent_scale_omegae.grid(row=1, column=1, padx=6, pady=3, sticky="w")
        self.ent_scale_omegac.grid(row=1, column=3, padx=6, pady=3, sticky="w")

        self.btn_lock_scales = tk.Button(row4, text="Lock Scales", width=12, command=self.on_toggle_lock_scales)
        self.btn_lock_scales.grid(row=1, column=5, padx=6, pady=3, sticky="e")

        # Row 5: Trigger controls (level override + delay)
        row5 = tk.LabelFrame(self, text="Trigger (OmegaElectrical)")
        row5.pack(fill="x", **pad)

        tk.Label(row5, text="Trigger level (RAW units):").grid(row=0, column=0, sticky="e", padx=6, pady=3)
        self.ent_trigger_level = tk.Entry(row5, width=14)
        self.ent_trigger_level.insert(0, "700")
        self.ent_trigger_level.config(state="disabled")
        self.ent_trigger_level.grid(row=0, column=1, padx=6, pady=3, sticky="w")

        tk.Label(row5, text="Trigger delay (% of window):").grid(row=0, column=2, sticky="e", padx=6, pady=3)
        self.ent_trigger_delay = tk.Entry(row5, width=6)
        self.ent_trigger_delay.insert(0, "10")
        self.ent_trigger_delay.config(state="disabled")
        self.ent_trigger_delay.grid(row=0, column=3, padx=6, pady=3, sticky="w")

        # Row 6: Status/output
        row6 = tk.Frame(self)
        row6.pack(fill="both", expand=True, **pad)
        self.txt_status = tk.Text(row6, height=16)
        self.txt_status.pack(fill="both", expand=True)
        self.safe_set_status(
            "Ready. Defaults: f=50 ⇒ Ts=2.5 ms. Currents scale=0.0003125, Speeds scale=0.19913 RPM/raw. "
            "Trigger fixed at raw 700 with 10% delay."
        )

        # Disable plot buttons if matplotlib missing
        if not HAS_PLT:
            self.btn_plot_curr.config(state="disabled")
            self.btn_plot_omega.config(state="disabled")

        # Close protocol
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Event Handlers ----------

    def on_browse_elf(self):
        path = filedialog.askopenfilename(title="Select ELF", filetypes=[("ELF files", "*.elf"), ("All files", "*.*")])
        if not path:
            return
        self.elf_path = Path(path)
        self.lbl_elf.config(text=str(self.elf_path.name))

    def _refresh_ports(self):
        ports = list_ports()
        menu = self.ddl_port["menu"]
        menu.delete(0, "end")
        if not ports:
            ports = [""]
        for p in ports:
            menu.add_command(label=p, command=lambda v=p: self.cbo_port.set(v))
        self.cbo_port.set(ports[0])

    def on_connect_toggle(self):
        if not self.connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        try:
            if not self.elf_path:
                messagebox.showwarning("Missing ELF", "Please select an ELF file first.")
                return
            port = self.cbo_port.get().strip()
            if not port:
                messagebox.showwarning("No Port", "Select a serial port.")
                return
            self.port = port
            self.baud = int(self.cbo_baud.get())

            # Create scope and import variables
            self.x2c = X2CScope(port=self.port, baudrate=self.baud, elf_file=str(self.elf_path))  # many builds accept this
            try:
                self.x2c.import_variables(str(self.elf_path))  # type: ignore (safe if already loaded)
            except Exception:
                pass

            self._resolve_handles()

            # Set app.hardwareUiEnabled = 0 once (best effort)
            h = self.handles.get(CTRL_HW_UI)
            if h is not None:
                try_write(self.x2c, h, 0)

            # Ensure device sampling factor is fixed to 50
            try:
                self.x2c.set_sample_time(FACTOR_F)  # type: ignore
            except Exception:
                pass

            self.connected = True
            self.btn_connect.config(text="Disconnect")
            self.safe_set_status(f"Connected on {self.port} @ {self.baud} baud. f=50 ⇒ Ts={TS_MS:.3f} ms, Fs≈{FS_HZ:.1f} Hz.")
        except Exception as e:
            self.connected = False
            self.x2c = None
            self.safe_show_error("Connection error", f"{e}\n\n{traceback.format_exc()}")

    def disconnect(self):
        # Do NOT write RUN/STOP here (one-shot semantics only on user actions)
        try:
            if self.x2c and hasattr(self.x2c, "close"):
                self.x2c.close()  # type: ignore
        except Exception:
            pass
        self.x2c = None
        self.connected = False
        self.btn_connect.config(text="Connect")
        self.safe_set_status("Disconnected.")

    def _resolve_handles(self):
        assert self.x2c is not None
        self.handles.clear()
        # Monitor
        for _lbl, sym in MONITOR_VARS:
            try:
                self.handles[sym] = self.x2c.get_variable(sym)  # type: ignore
            except Exception:
                self.handles[sym] = None
        # Control
        for sym in (CTRL_HW_UI, CTRL_VEL_REF, CTRL_RUN_REQ, CTRL_STOP_REQ):
            try:
                self.handles[sym] = self.x2c.get_variable(sym)  # type: ignore
            except Exception:
                self.handles[sym] = None

    def on_test_sampling(self):
        """Query total ms using get_scope_sample_time(50 µs) with f=50 already set."""
        if not self.connected or not self.x2c:
            self.safe_set_status("Not connected. Connect first to test sampling.")
            return

        # Reinforce f=50 just before test
        try:
            self.x2c.set_sample_time(FACTOR_F)  # type: ignore
        except Exception:
            pass

        total_ms = 0.0
        try:
            total_ms = get_total_ms_via_api(self.x2c, RAW_SAMPLE_TIME_US)
        except Exception:
            total_ms = 0.0
        origin = "device API"
        if total_ms <= 0.0:
            total_ms = TOTAL_MS_EXPECTED
            origin = "expected"

        msg = (
            f"[Sampling Test] f=50, raw=50 µs → total≈{total_ms:.2f} ms ({origin}). "
            f"Effective Ts is FIXED at {TS_MS:.3f} ms per sample (Fs≈{FS_HZ:.1f} Hz)."
        )
        self.safe_set_status(msg)

    def _read_scales(self) -> Dict[str, float]:
        """Get per-channel scaling factors from UI (real = raw × scale)."""
        return {
            "idqCmd_q":        safe_float(self.ent_scale_idqcmd.get(), DEFAULT_SCALES["idqCmd_q"]),
            "Idq_q":           safe_float(self.ent_scale_idqq.get(),   DEFAULT_SCALES["Idq_q"]),
            "Idq_d":           safe_float(self.ent_scale_idqd.get(),   DEFAULT_SCALES["Idq_d"]),
            "OmegaElectrical": safe_float(self.ent_scale_omegae.get(), DEFAULT_SCALES["OmegaElectrical"]),  # RPM per raw
            "OmegaCmd":        safe_float(self.ent_scale_omegac.get(), DEFAULT_SCALES["OmegaCmd"]),
        }

    def on_toggle_lock_scales(self):
        """Lock/unlock the scaling entries to 'fix' values before the run."""
        self.scales_locked = not self.scales_locked
        state = "disabled" if self.scales_locked else "normal"
        for ent in (self.ent_scale_idqcmd, self.ent_scale_idqq, self.ent_scale_idqd,
                    self.ent_scale_omegae, self.ent_scale_omegac):
            ent.config(state=state)
        self.btn_lock_scales.config(text=("Unlock Scales" if self.scales_locked else "Lock Scales"))
        self.safe_set_status(("Scales locked." if self.scales_locked else "Scales unlocked."))

    def on_start(self):
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("Busy", "Capture already running.")
            return

        # Connect on demand
        if not self.connected:
            self.connect()
            if not self.connected:
                return

        # Parameters: convert RPM to counts via Scale; trigger level computed from controls
        speed_rpm = safe_float(self.ent_speed.get(), 0.0)
        vel_scale_rpm_per_count = safe_float(self.ent_vel_scale.get(), 0.0)
        if vel_scale_rpm_per_count == 0.0:
            messagebox.showwarning("Invalid Scale", "Scale (RPM per count) must be non-zero.")
            return

        counts = speed_rpm / vel_scale_rpm_per_count

        scales = self._read_scales()  # either locked or not, we just read current values

        # Trigger settings
        trigger_level_raw = safe_float(self.ent_trigger_level.get(), 0.0)
        trigger_delay_pct = safe_float(self.ent_trigger_delay.get(), 10.0)

        # Warn if some variables are missing
        missing = [sym for (_lbl, sym) in MONITOR_VARS if self.handles.get(sym) is None]
        if missing:
            if not messagebox.askyesno(
                "Missing variables",
                "Some scope variables could not be resolved:\n"
                + "\n".join(missing)
                + "\n\nStart capture anyway?"
            ):
                return

        # Start worker (no auto-save)
        self.worker = CaptureWorker(
            app_ref=self,
            x2c=self.x2c,  # type: ignore
            handles=self.handles,
            variables=MONITOR_VARS,
            counts=counts,
            trigger_raw_level=trigger_level_raw,
            trigger_delay_pct=int(trigger_delay_pct),
            scales=scales,
        )
        self.worker.start()
        self.safe_set_status(
            f"RUN will be sent once. Trigger: OmegaElectrical rising to raw≈{trigger_level_raw:.1f}, "
            f"delay={int(trigger_delay_pct)}%. Capture window ~1.225 s at 2.5 ms/sample…"
        )
        # Log current scale set for traceability
        self.safe_set_status(
            "Scales used: " + ", ".join([f"{k}={v}" for k, v in scales.items()])
        )

    def on_stop(self):
        # Early stop: one-shot STOP + finish any waiting
        if self.worker and self.worker.is_alive():
            self.worker.stop_early()
            self.safe_set_status("Stop requested (one-shot).")
        else:
            # Best-effort one-shot STOP even if no worker
            if self.connected and self.x2c:
                h = self.handles.get(CTRL_STOP_REQ)
                if h is not None:
                    try_write(self.x2c, h, 1)
                    self.safe_set_status("STOP sent.")

    def on_export(self):
        """Export the most recent triggered capture (**SCALED**) to a user-selected TSV path."""
        if not self.last_scaled or not self.last_t:
            messagebox.showinfo("No data", "Run a capture first.")
            return
        path = filedialog.asksaveasfilename(
            title="Save TSV (scaled values)",
            defaultextension=".tsv",
            filetypes=[("TSV files", "*.tsv"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            self._save_tsv(Path(path), self.last_t, self.last_scaled)
            self.safe_set_status(f"Exported: {Path(path).resolve()}")
        except Exception as e:
            self.safe_show_error("Export error", f"{e}\n\n{traceback.format_exc()}")

    def _save_tsv(self, tsv_path: Path, t_axis: List[float], data_scaled: Dict[str, List[float]]):
        """Write scaled values to a tab-separated file with exact label ordering."""
        # Align rows to the shortest column length
        n_min = min([len(t_axis)] + [len(data_scaled.get(lbl, [])) for (lbl, _s) in MONITOR_VARS]) if data_scaled else 0
        headers = ["t_s"] + [lbl for (lbl, _s) in MONITOR_VARS]
        with tsv_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f, delimiter="\t")
            w.writerow(headers)
            for i in range(n_min):
                row = [t_axis[i]] + [data_scaled.get(lbl, [None]*n_min)[i] for (lbl, _s) in MONITOR_VARS]
                w.writerow(row)

    def on_capture_complete(
        self,
        data_raw: Dict[str, List[float]],
        t_axis: List[float],
        total_ms: float,
        scales: Dict[str, float],
    ):
        # Store raw and scales
        self.last_raw = data_raw
        self.last_t = t_axis
        self.last_total_ms = total_ms
        self.last_scales = scales

        # Build scaled series for plotting/export: real = raw × scale
        scaled: Dict[str, List[float]] = {}
        n_min = min((len(v) for v in data_raw.values()), default=0)
        for lbl, _sym in MONITOR_VARS:
            s = scales.get(lbl, 1.0)
            raw_series = data_raw.get(lbl, [])[:n_min]
            scaled[lbl] = [x * s for x in raw_series]
        self.last_scaled = scaled

        self.safe_set_status(
            f"Triggered capture complete. Total≈{total_ms:.2f} ms, Ts={TS_MS:.3f} ms (Fs≈{FS_HZ:.1f} Hz). "
            f"Use 'Export…' to save TSV (scaled)."
        )
        # Enable plot buttons if matplotlib available & data present
        has_any = t_axis and any(len(v) for v in scaled.values())
        if HAS_PLT and has_any:
            self.btn_plot_curr.config(state="normal")
            self.btn_plot_omega.config(state="normal")

    # ---------- Plotting (scaled) ----------

    def show_plot(self, group: str):
        if not HAS_PLT:
            self.safe_show_error("Plotting unavailable", "matplotlib is not installed.")
            return
        if not self.last_scaled or not self.last_t:
            messagebox.showinfo("No data", "Run a capture first.")
            return

        t = self.last_t
        if group == "currents":
            keys = ["idqCmd_q", "Idq_q", "Idq_d"]
            title = "Currents (scaled)"
            ylab = "Current (scaled)"
        else:
            keys = ["OmegaElectrical", "OmegaCmd"]
            title = "Omega (scaled)"
            ylab = "Speed (RPM)"

        fig, ax = plt.subplots()
        for k in keys:
            if k in self.last_scaled:
                ax.plot(t, self.last_scaled[k], label=k)
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylab)
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        plt.show()

    # ---------- Helpers ----------

    def safe_set_status(self, text: str):
        def _do():
            self.txt_status.insert("end", text + "\n")
            self.txt_status.see("end")
        self.after(0, _do)

    def safe_show_error(self, title: str, message: str):
        self.after(0, lambda: messagebox.showerror(title, message))

    def on_close(self):
        try:
            if self.worker and self.worker.is_alive():
                self.worker.stop_early()
                time.sleep(0.2)  # give it a moment to send STOP
        except Exception:
            pass
        try:
            self.disconnect()
        except Exception:
            pass
        self.destroy()


# ----------------------------- Entry Point -----------------------------

def main():
    app = MotorLoggerApp()
    app.mainloop()


if __name__ == "__main__":
    main()
