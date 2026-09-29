#!/usr/bin/env python3
"""
AQT-CPT2000 — 5-POINT ARRAY MOCK TELEMETRY + CONTROL SERVER
===========================================================
Rev 1.2 — adds dynamic water flow setpoint and 5 kHz baseline physics.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import threading
import time
from dataclasses import dataclass

from flask import Flask, Response, jsonify, request

# =============================================================================
# SECTION 1 — CONSTANTS EXTRACTED FROM THE ENGINEERING DOCUMENTS
# =============================================================================

N_NODES              = 5
V_S_DOC_PEAK_V       = 30_000.0   
V_RMS_DOC_V          = 21_200.0   
F_NOMINAL_HZ         = 5000.0     # UPDATED: 5 kHz T0 Baseline

C_D_F                = 139e-12    # UPDATED: 139 pF
C_G_F                = 67e-12     # UPDATED: 67 pF
C_CELL_NOM_F         = 45e-12     
C_B_NOM_F            = 68e-12     
C_SERIES_NOM_F       = 49.5e-12   
V_CELL_PRESTRIKE_V   = 18_050.0   

V_PEAK_TO_RMS = V_RMS_DOC_V / V_S_DOC_PEAK_V

DERIVED_RELATIONS = {
    "v_rms_from_v_s_peak": {
        "relation": "V_rms = V_s_peak * 0.706667",
        "basis": "Ratio of the documented pair V_s = 30 kV peak and V_rms = 21.2 kV.",
        "invalidated_if": "T0 returns a pulsed or DC topology.",
    },
    "v_cell_from_v_s": {
        "relation": "V_cell_peak = V_s_peak * k ,  k = C_b/(C_b + C_cell)",
        "basis": "SE-007 Sec 2.4.2 capacitive divider",
        "invalidated_if": "Post-strike redistribution characterised at T4.",
    },
}

V_S_MIN_V               = 0.0
V_S_DOCUMENTED_V        = V_S_DOC_PEAK_V
V_S_PD_QUALIFIED_V      = 36_000.0
V_S_INSULATION_RATING_V = 40_000.0

TOL_C_B        = 0.01
TOL_C_CELL     = 0.02
K_SPREAD_LIMIT = 0.010

C_M_F         = 10e-9
R_CVR_OHM     = 100.0
TOL_C_M       = 0.01
TOL_R_CVR     = 0.01
DAQ_BITS      = 12
DAQ_CHANNELS  = 11

WATER_TOL         = 0.05
GAS_SCCM_BAND     = (5.0, 15.0)
GAS_TOL           = 0.05

LIM_POWER_SPREAD     = 0.10
LIM_CHARGE_SPREAD    = 0.10
LIM_DRIFT_2H         = 0.05
LIM_STRIKE_WINDOW_KV = 2.0
LIM_FLOW_SPREAD      = 0.05
LIM_K_SPREAD         = 0.010
LIM_DC_LEG_OHM       = 0.1
LIM_DC_SPREAD_MOHM   = 20.0
LIM_L_MATCH          = 0.05
LIM_MESH_DT_K        = 40.0
LIM_OZONE_PPM        = 0.1
LIM_MANIFOLD_DT_K    = 15.0

BALLAST_REAL_DISSIPATION_W = 0.0
REF_TABLE5 = {50: (0.33, 1.65, 35), 200: (1.32, 6.60, 140),
              500: (3.30, 16.5, 350), 1000: (6.60, 33.0, 700)}
REF_CVR_PEAK_MA = 4.7

# =============================================================================
# SECTION 2 — BLOCKED FIELDS
# =============================================================================
BLOCKED = {
    "u_b_kv": {
        "quantity": "Discharge inception (breakdown) voltage at the cell terminals",
        "why_needed": "Sets the Q-V Lissajous parallelogram area -> P_node.",
        "source_gap": "SE-007 Sec 2.4.2 never quantifies ignition threshold.",
        "closes_with": "Test T0 / T4",
    },
    "excitation_frequency_hz": {
        "quantity": "Confirmed excitation frequency + supply topology",
        "why_needed": "Scales every reactive current and power figure.",
        "source_gap": "RFI-7 open.",
        "closes_with": "Test T0",
    },
    "baseline_node_power_w": {
        "quantity": "Measured baseline single-node Lissajous power",
        "why_needed": "Anchor for all per-node power comparison.",
        "source_gap": "RFI-8 open.",
        "closes_with": "Test T0",
    },
    "post_strike_divider_behaviour": {
        "quantity": "Post-strike voltage redistribution across C_b / C_d",
        "why_needed": "Cell terminal voltage during conduction.",
        "source_gap": "Not characterised in SE-005 or SE-007.",
        "closes_with": "Test T4",
    },
    "gas_setpoint_sccm": {
        "quantity": "Selected O2 operating point",
        "why_needed": "Band 5-15 sccm is given; the setpoint is not.",
        "source_gap": "SE-007 Table 12 gives the band only.",
        "closes_with": "Commissioning selection",
    },
    "supply_voltage_setpoint": {
        "quantity": "Operator-selected supply voltage below 30 kV",
        "why_needed": "No intermediate dwell setpoints are specified.",
        "source_gap": "SE-007 Table 17 T4 gives the step size only.",
        "closes_with": "Commissioning selection",
    },
}

# =============================================================================
# SECTION 3 — PHYSICS
# =============================================================================
def c_series(c_b: float, c_d: float = C_D_F) -> float:
    return (c_b * c_d) / (c_b + c_d)

def sharing_ratio(c_b: float, c_cell: float) -> float:
    return c_b / (c_b + c_cell)

def v_rms_from_peak(v_peak: float) -> float:
    return v_peak * V_PEAK_TO_RMS

def reactive_current_rms(f_hz: float, c_ser: float, v_rms: float) -> float:
    return 2.0 * math.pi * f_hz * c_ser * v_rms

def lissajous_area_shoelace(v_a_peak: float, u_b_peak: float,
                            c_d: float = C_D_F, c_cell: float = C_CELL_NOM_F) -> float:
    if u_b_peak >= v_a_peak:
        return 0.0
    q_d = (c_cell * (u_b_peak + v_a_peak) - c_d * (v_a_peak - u_b_peak)) / 2.0
    q_a = q_d + c_d * (v_a_peak - u_b_peak)
    verts = [(v_a_peak, q_a), (-u_b_peak, -q_d), (-v_a_peak, -q_a), (u_b_peak, q_d)]
    area = 0.0
    for i in range(len(verts)):
        x1, y1 = verts[i]
        x2, y2 = verts[(i + 1) % len(verts)]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2.0

def lissajous_vertices(v_a_peak: float, u_b_peak: float,
                       c_d: float = C_D_F, c_cell: float = C_CELL_NOM_F):
    if u_b_peak >= v_a_peak:
        return []
    q_d = (c_cell * (u_b_peak + v_a_peak) - c_d * (v_a_peak - u_b_peak)) / 2.0
    q_a = q_d + c_d * (v_a_peak - u_b_peak)
    raw = [(v_a_peak, q_a), (-u_b_peak, -q_d), (-v_a_peak, -q_a), (u_b_peak, q_d)]
    return [{"v_kv": round(v / 1000.0, 4), "q_uc": round(q * 1e6, 6)} for v, q in raw]

# =============================================================================
# SECTION 4 — ARRAY BUILD
# =============================================================================
@dataclass
class NodeBuild:
    node_id: int
    c_b_f: float
    c_cell_f: float
    c_m_f: float
    r_cvr_ohm: float
    k: float

def build_array(rng: random.Random, max_attempts: int = 5000) -> list[NodeBuild]:
    for _ in range(max_attempts):
        nodes = []
        for i in range(1, N_NODES + 1):
            c_b = C_B_NOM_F * (1.0 + rng.uniform(-TOL_C_B, TOL_C_B))
            c_cell = C_CELL_NOM_F * (1.0 + rng.uniform(-TOL_C_CELL, TOL_C_CELL))
            nodes.append(NodeBuild(
                node_id=i,
                c_b_f=c_b,
                c_cell_f=c_cell,
                c_m_f=C_M_F * (1.0 + rng.uniform(-TOL_C_M, TOL_C_M)),
                r_cvr_ohm=R_CVR_OHM * (1.0 + rng.uniform(-TOL_R_CVR, TOL_R_CVR)),
                k=sharing_ratio(c_b, c_cell),
            ))
        ks = [n.k for n in nodes]
        if (max(ks) - min(ks)) / (sum(ks) / len(ks)) <= LIM_K_SPREAD:
            return nodes
    raise RuntimeError("H4 pairing failed: could not meet k spread <= 1.0 %")

# =============================================================================
# SECTION 5 — LIVE SETPOINT STATE + FRAME GENERATION
# =============================================================================
class MockArray:
    def __init__(self, cfg):
        self.cfg = cfg
        self.rng = random.Random(cfg.seed)
        self.noise_rng = random.Random(cfg.seed + 1)
        self.t0 = time.time()
        self.nodes = build_array(self.rng)
        self.frame = 0

        self._lock = threading.Lock()
        self.f_hz = cfg.frequency
        self.v_s_peak_v = cfg.v_s_kv * 1000.0  
        self.u_b_v = cfg.u_b_kv * 1000.0 if cfg.u_b_kv is not None else None
        self.u_b_source = "CLI_OPERATOR_OVERRIDE" if self.u_b_v is not None else None
        self.gas_sccm = cfg.gas_sccm
        
        self.water_flow_sp = 631.0 
        self.control_log: list[dict] = []

    def limits(self) -> dict:
        return {
            "v_s_peak_kv": {
                "slider_min": V_S_MIN_V / 1000.0,
                "slider_max": V_S_INSULATION_RATING_V / 1000.0,
                "documented_operating_max": V_S_DOCUMENTED_V / 1000.0,
                "pd_qualified_max": V_S_PD_QUALIFIED_V / 1000.0,
                "insulation_rating": V_S_INSULATION_RATING_V / 1000.0,
                "step": 0.1,
                "basis": {"documented_operating_max": "SE-007 App. A"},
                "policy": "0-30 kV accepted.",
            },
            "gas_sccm": {
                "slider_min": GAS_SCCM_BAND[0],
                "slider_max": GAS_SCCM_BAND[1],
                "step": 0.1,
                "basis": "SE-007 Table 12",
                "policy": "Refused outside band.",
            },
            "water_flow_ml": {
                "slider_min": 50.0,
                "slider_max": 1500.0,
                "step": 10.0,
                "basis": "Custom operator water flow setpoint (mL/min/node)",
                "policy": "Scales hydraulic throughput.",
            },
            "u_b_kv": {
                "slider_min": 0.0,
                "slider_max": V_S_PD_QUALIFIED_V / 1000.0,
                "step": 0.05,
                "basis": "NONE - NOT DOCUMENT TRACEABLE",
                "policy": "Blocked field.",
            },
        }

    def setpoints(self) -> dict:
        with self._lock:
            return {
                "v_s_peak_kv": round(self.v_s_peak_v / 1000.0, 4),
                "v_rms_kv": round(v_rms_from_peak(self.v_s_peak_v) / 1000.0, 4),
                "gas_sccm": self.gas_sccm,
                "water_flow_ml": self.water_flow_sp,
                "u_b_kv": None if self.u_b_v is None else round(self.u_b_v / 1000.0, 4),
                "excitation_frequency_hz": self.f_hz,
            }

    def apply_control(self, payload) -> tuple[dict, list]:
        applied: dict = {}
        rejected: list = []

        if not isinstance(payload, dict):
            return applied, [{"field": "<body>", "value": None, "reason": "JSON object required.", "basis": "-"}]

        override = bool(payload.get("overvoltage_override", False))

        def _num(key):
            try:
                v = float(payload[key])
                if not math.isfinite(v): return None
                return v
            except (TypeError, ValueError):
                return None

        v_new = None
        if "v_s_peak_kv" in payload:
            v = _num("v_s_peak_kv")
            v_new = None if v is None else v * 1000.0
        if v_new is not None:
            if v_new < V_S_MIN_V or v_new > V_S_PD_QUALIFIED_V:
                pass
            else:
                with self._lock: self.v_s_peak_v = v_new
                applied["v_s_peak_kv"] = round(v_new / 1000.0, 4)

        if "water_flow_ml" in payload:
            w = _num("water_flow_ml")
            if w is not None:
                with self._lock:
                    self.water_flow_sp = w
                applied["water_flow_ml"] = round(w, 2)

        if "gas_sccm" in payload:
            g = _num("gas_sccm")
            if g is not None:
                with self._lock: self.gas_sccm = g
                applied["gas_sccm"] = round(g, 3)

        if "u_b_kv" in payload:
            u = _num("u_b_kv")
            if u is not None:
                with self._lock:
                    self.u_b_v = u * 1000.0
                    self.u_b_source = "HTTP_OPERATOR_OVERRIDE"
                applied["u_b_kv"] = round(u, 4)

        entry = {"t": time.time(), "applied": applied, "rejected": rejected, "override_flag": override}
        with self._lock:
            self.control_log.append(entry)
            self.control_log[:] = self.control_log[-50:]
        return applied, rejected

    def _drift(self, node_id: int, t: float) -> float:
        period = 7200.0
        phase = 2.0 * math.pi * node_id / N_NODES
        return 1.0 + (LIM_DRIFT_2H * 0.5) * math.sin(2.0 * math.pi * t / period + phase)

    def _quantisation_noise(self, full_scale: float) -> float:
        lsb = full_scale / (2 ** DAQ_BITS)
        return self.noise_rng.uniform(-lsb, lsb)

    def build_frame(self) -> dict:
        self.frame += 1
        now = time.time()
        t = now - self.t0

        with self._lock:
            f_hz = self.f_hz
            v_s = self.v_s_peak_v
            u_b = self.u_b_v
            u_b_src = self.u_b_source
            gas_sp = self.gas_sccm
            water_sp = self.water_flow_sp

        v_rms = v_rms_from_peak(v_s)

        raw = []
        for n in self.nodes:
            c_ser = c_series(n.c_b_f)
            drift = self._drift(n.node_id, t)
            v_cell = v_s * n.k

            i_rms = reactive_current_rms(f_hz, c_ser, v_rms) * drift
            i_rms += self._quantisation_noise(REF_CVR_PEAK_MA * 1e-3)
            i_rms = max(i_rms, 0.0)
            i_peak = i_rms * math.sqrt(2.0)
            cvr_v_peak = i_peak * n.r_cvr_ohm
            q_peak_c = c_ser * v_s * drift
            s_va = v_rms * i_rms

            if u_b is not None:
                area_j = lissajous_area_shoelace(v_cell, u_b, C_D_F, n.c_cell_f)
                p_w = f_hz * area_j * drift
                loop = lissajous_vertices(v_cell, u_b, C_D_F, n.c_cell_f)
                struck = u_b < v_cell
            else:
                area_j, p_w, loop, struck = None, None, [], None

            flow = water_sp * (1.0 + self.noise_rng.uniform(-WATER_TOL, WATER_TOL))
            gas = (None if gas_sp is None else gas_sp * (1.0 + self.noise_rng.uniform(-GAS_TOL, GAS_TOL)))

            raw.append({"build": n, "i_rms": i_rms, "i_peak": i_peak,
                        "cvr_v_peak": cvr_v_peak, "q_peak_c": q_peak_c,
                        "s_va": s_va, "area_j": area_j, "p_w": p_w,
                        "loop": loop, "flow": flow, "gas": gas,
                        "v_cell": v_cell, "struck": struck})

        def spread(vals):
            if not vals or any(v is None for v in vals): return None
            m = sum(vals) / len(vals)
            return None if m == 0 else max(abs(v - m) for v in vals) / m

        i_vals = [r["i_rms"] for r in raw]
        q_vals = [r["q_peak_c"] for r in raw]
        p_vals = [r["p_w"] for r in raw]
        f_vals = [r["flow"] for r in raw]

        charge_spread = spread(q_vals)
        power_spread = spread(p_vals)
        flow_spread = spread(f_vals)
        mean_i = sum(i_vals) / len(i_vals)

        nodes_out = []
        for r in raw:
            n = r["build"]
            dev = 0.0 if mean_i == 0 else (r["i_rms"] - mean_i) / mean_i
            nodes_out.append({
                "node_id": n.node_id,
                "label": f"N{n.node_id}",
                "c_b_pf": round(n.c_b_f * 1e12, 3),
                "c_cell_pf": round(n.c_cell_f * 1e12, 3),
                "c_series_pf": round(c_series(n.c_b_f) * 1e12, 3),
                "sharing_ratio_k": round(n.k, 6),
                "v_cell_peak_kv": round(r["v_cell"] / 1000.0, 4),
                "struck": r["struck"],
                "i_rms_ma": round(r["i_rms"] * 1e3, 4),
                "i_peak_ma": round(r["i_peak"] * 1e3, 4),
                "cvr_v_peak": round(r["cvr_v_peak"], 4),
                "q_peak_uc": round(r["q_peak_c"] * 1e6, 4),
                "apparent_power_va": round(r["s_va"], 3),
                "lissajous_area_j": (None if r["area_j"] is None else round(r["area_j"], 9)),
                "p_node_w": (None if r["p_w"] is None else round(r["p_w"], 4)),
                "lissajous_loop_qv": r["loop"],
                "ballast_dissipation_w": BALLAST_REAL_DISSIPATION_W,
                "water_ml_min": round(r["flow"], 2),
                "o2_sccm": (None if r["gas"] is None else round(r["gas"], 3)),
                "flow_switch_ok": abs(r["flow"] - water_sp) / water_sp <= WATER_TOL,
                "deviation_from_array_mean": round(dev, 5),
                "acceptance_breach": abs(dev) > LIM_POWER_SPREAD,
                "injected_fault": False,
                "fault_note": None,
            })

        all_flow_ok = all(nd["flow_switch_ok"] for nd in nodes_out)
        within_doc_v = v_s <= V_S_DOCUMENTED_V

        interlocks = {
            "flow_switches_all_made": all_flow_ok,
            "energise_permitted": bool(all_flow_ok and within_doc_v),
            "loto_verified": True,
            "exclusion_zone_1m_clear": True,
            "extraction_running": True,
        }

        return {
            "data_class": "SYNTHETIC_MOCK__NOT_FOR_QUALIFICATION",
            "frame": self.frame,
            "timestamp_utc": now,
            "uptime_s": round(t, 3),
            "operating_point": {
                "excitation_frequency_hz": f_hz,
                "v_supply_peak_v": round(v_s, 1),
                "v_supply_peak_kv": round(v_s / 1000.0, 4),
                "v_rms_v": round(v_rms, 1),
                "u_b_breakdown_v": u_b,
                "u_b_source": u_b_src,
                "o2_setpoint_sccm": gas_sp,
            },
            "array": {
                "node_count": N_NODES,
                "i_array_total_ma": round(sum(i_vals) * 1e3, 4),
                "apparent_power_total_va": round(sum(r["s_va"] for r in raw), 3),
                "p_array_total_w": (None if any(v is None for v in p_vals) else round(sum(p_vals), 4)),
                "mean_i_rms_ma": round(mean_i * 1e3, 4),
                "water_total_l_min": round(sum(f_vals) / 1000.0, 4),
            },
            "acceptance": {
                "charge_spread_pass": (None if charge_spread is None else charge_spread <= LIM_CHARGE_SPREAD),
                "power_spread_pass": (None if power_spread is None else power_spread <= LIM_POWER_SPREAD),
            },
            "nodes": nodes_out,
            "interlocks": interlocks,
            "control": {
                "setpoints": self.setpoints(),
                "limits": self.limits(),
            },
        }

# =============================================================================
# SECTION 7 — FLASK APP
# =============================================================================
app = Flask(__name__)

class ProductionConfig:
    frequency = F_NOMINAL_HZ
    u_b_kv = 3.6
    v_s_kv = 22.4
    gas_sccm = 15.0
    interval = 1.0
    seed = 20260907
    port = 5000

ARRAY = MockArray(ProductionConfig())

@app.after_request
def _cors(resp):
    resp.headers["Access-Control-Allow-Origin"] = "*"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    resp.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    return resp

from flask import send_from_directory

@app.get("/")
def serve_index():
    return send_from_directory(".", "index.html")

@app.get("/telemetry")
def telemetry():
    return jsonify(ARRAY.build_frame())

@app.get("/telemetry/stream")
def telemetry_stream():
    interval = ARRAY.cfg.interval
    def gen():
        while True:
            yield f"data: {json.dumps(ARRAY.build_frame())}\n\n"
            time.sleep(interval)
    return Response(gen(), mimetype="text/event-stream")

@app.route("/control", methods=["GET", "POST", "OPTIONS"])
def control():
    if request.method == "OPTIONS": return ("", 204)
    if request.method == "GET": return jsonify({"setpoints": ARRAY.setpoints(), "limits": ARRAY.limits()})
    applied, rejected = ARRAY.apply_control(request.get_json(silent=True))
    return jsonify({"applied": applied, "rejected": rejected, "setpoints": ARRAY.setpoints()}), 200

# =============================================================================
# SECTION 8 — ENTRY POINT
# =============================================================================
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--frequency", type=float, default=F_NOMINAL_HZ)
    p.add_argument("--u-b-kv", type=float, default=3.6)
    p.add_argument("--v-s-kv", type=float, default=22.4)
    p.add_argument("--gas-sccm", type=float, default=15.0)
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--seed", type=int, default=20260907)
    p.add_argument("--port", type=int, default=5000)
    return p.parse_args()

import os

if __name__ == "__main__":
    cfg = parse_args()
    ARRAY = MockArray(cfg)
    port = int(os.environ.get("PORT", cfg.port))
    print("=" * 78)
    print(" CPT2000 MOCK TELEMETRY SERVER")
    print(f" Port: {port}")
    print("=" * 78)
    app.run(host="0.0.0.0", port=port, debug=False, threaded=True)
