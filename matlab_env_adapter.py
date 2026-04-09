"""
MatlabEnvAdapter
================
Drop-in replacement for DeterministicToy5GEnvAdapter.
Wraps TCP communication với MATLAB (nrDRLScheduler.m ở TrainingMode).

Interface is the same with toy5g for no change train_matlab.py despite of create env.

Protocol (layer-by-layer):
  MATLAB → Python : TTI_START  {tti}
  for l in 0..n_layers-1:
    MATLAB → Python : LAYER_OBS     {layer, obs:[obs_dim], masks:[n_rbg][act_dim]}
    Python → MATLAB : LAYER_ACT     {actions:[n_rbg]}   (0-indexed local UE / noop)
    MATLAB → Python : LAYER_REWARD  {rewards:[n_rbg], next_obs:[obs_dim], next_masks:[n_rbg][act_dim]}
    MATLAB → Python : TTI_DONE      {metrics:{avg_cell_tput, jain, pf_utility,
                                         avg_layers_per_rbg, no_schedule_rate}}

obs_dim  = (5 + 2*n_rbg) * max_sched_ue   (matches toy5g _build_obs)
act_dim  = max_sched_ue + 1               (matches toy5g act_dim)
noop     = max_sched_ue                    (matches toy5g noop)
"""

from __future__ import annotations

import json
import socket
from dataclasses import dataclass
from typing import Iterator, List, Dict, Optional, Tuple
import math
from pprint import pprint
import torch
import numpy as np

# from nr_utils import (
#     CQI_TO_MCS
# )

CQI_TO_MCS = [0, 0, 1, 3, 5, 7, 9, 11, 13, 15, 18, 20, 22, 24, 26, 28]

MCS_TABLE: List[Tuple[int, int]] = [
    (2, 120), (2, 157), (2, 193), (2, 251), (2, 308), (2, 379),
    (4, 449), (4, 526), (4, 602), (4, 679), (6, 340), (6, 378),
    (6, 434), (6, 490), (6, 553), (6, 616), (6, 658), (8, 438),
    (8, 466), (8, 517), (8, 567), (8, 616), (8, 666), (8, 719),
    (8, 772), (8, 822), (8, 873), (8, 910), (8, 948),
]


def tbs_38214_bytes(mcs_idx, n_prb, n_symb=14, n_layers=1, overhead_re_per_prb=18):
    """Very light TBS proxy (bytes)."""
    if n_prb <= 0 or mcs_idx < 0:
        return 0
    mcs_idx = int(max(0, min(28, mcs_idx)))
    Qm, R1024 = MCS_TABLE[mcs_idx]
    R = R1024 / 1024.0
    N_re_per_prb = max(0, 12 * n_symb - overhead_re_per_prb)
    Ninfo = R * Qm * N_re_per_prb * int(n_prb) * n_layers
    if Ninfo <= 0:
        return 0
    if Ninfo <= 3824:
        TBS_bits = int(6 * math.ceil(Ninfo / 6.0))
    else:
        TBS_bits = int(6 * math.ceil((Ninfo - 24) / 6.0))
    TBS_bits = max(TBS_bits, 24)
    return TBS_bits // 8

# Reuse LayerContext dataclass definition (identical to toy5g)
@dataclass
class LayerContext:
    layer: int
    obs: torch.Tensor        # [obs_dim]
    masks_rbg: torch.Tensor  # [n_rbg, act_dim] bool


# ─── TCP helpers ──────────────────────────────────────────────────────────────

_NL = b"\n"


def _recv_json(sock: socket.socket, rxbuf: bytearray) -> dict:
    """Block until one newline-delimited JSON message is available."""
    while True:
        idx = rxbuf.find(_NL)
        if idx != -1:
            line = bytes(rxbuf[:idx])
            del rxbuf[:idx + 1]
            if not line.strip():
                continue
            return json.loads(line.decode("utf-8"))
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("[MatlabEnv] MATLAB disconnected unexpectedly")
        rxbuf.extend(chunk)

def _send_json(sock: socket.socket, obj: dict) -> None:
    """Send a single JSON object with newline terminator."""
    data = (json.dumps(obj) + "\n").encode("utf-8")
    sock.sendall(data)


# ─── Adapter ──────────────────────────────────────────────────────────────────

class MatlabEnvAdapter:
    """
    Wraps MATLAB simulation as an RL environment with the same interface
    as DeterministicToy5GEnvAdapter.

    Key attributes (mirrors toy5g):
        max_sched_ue : int   – eligible UEs per TTI (= MATLAB MaxUEs, default 16)
        n_ue         : int   – same as max_sched_ue (for eval compatibility)
        n_layers     : int   – MU-MIMO spatial layers (= MATLAB NumLayers)
        n_rbg        : int   – Resource Block Groups
        obs_dim      : int   – (5 + 2*n_rbg) * max_sched_ue
        act_dim      : int   – max_sched_ue + 1
        noop         : int   – max_sched_ue  (no-allocation action)
        eps          : float – small constant
        _alloc       : Tensor[n_layers, n_rbg]  – current TTI allocation
        _last_metrics: dict  – metrics received from MATLAB after finish_tti()
    """
    def __init__(
        self,
        *,
        max_sched_ue: int = 16,
        n_layers: int = 16,
        n_rbg: int = 18,
        port: int = 5555,
        host: str = "0.0.0.0",
        eps: float = 1e-6,
        device: str = "cpu",
        verbose: bool = True,
        tti_ms: float = 0.5,
        prbs_per_rbg: int = 18,
        n_symb: int = 14,
        overhead_re_per_prb: int = 18,
        internal_log: bool = False,
        max_mcs: int = 28,
        max_cqi: int = 15,
        max_ue_rank: int = 2,
        max_rate: float = 1100.0,
        ema_beta: float = 0.9,
        strict_eligibility_assert: bool = True,
        mcs_spread: int = 1,
        seed: int = 0,
    ):
        self.max_sched_ue = max_sched_ue
        self.n_ue = max_sched_ue          # alias for eval compatibility
        self.n_layers = n_layers
        self.n_rbg = n_rbg
        self.noop = max_sched_ue          # same as toy5g
        self.act_dim = max_sched_ue + 1   # same as toy5g
        self.obs_dim = (5 + 2 * n_rbg) * max_sched_ue

        self.device = torch.device(device)
        self.eps = float(eps)
        self.verbose = verbose
        self.internal_log = internal_log
        self.ema_beta = ema_beta
        self.tti_ms = float(tti_ms)
        self.prbs_per_rbg = int(prbs_per_rbg)
        self.n_symb = int(n_symb)
        self.overhead = int(overhead_re_per_prb)
        self.max_mcs = int(max_mcs)
        self.max_cqi = int(max_cqi)
        self.max_ue_rank = int(max_ue_rank)
        self.max_rate = float(max_rate)
        self.strict_eligibility_assert = bool(strict_eligibility_assert)
        self.mcs_spread = max(0, int(mcs_spread))
        self.rng = np.random.default_rng(seed)

        # Internal state (mirrors toy5g)
        self._alloc = torch.full(
            (n_layers, n_rbg), self.noop, device=self.device, dtype=torch.long
        )
        self._cur_layer: Optional[int] = None
        self._last_metrics: dict = {}

        self.t = 0
        self.buf = torch.full((self.n_ue,), 0.0, device=self.device, dtype=torch.float32)  # bytes
        self.avg_tp = torch.full((self.n_ue,), self.eps, device=self.device, dtype=torch.float32)    # EMA throughput (Mbps)
        self.ue_priority = torch.full((self.n_ue,), 0, device=self.device, dtype=torch.int)
        self.ue_rank = torch.full((self.n_ue,),2,device=self.device)
        self.wb_cqi = torch.full((self.n_ue,),2,device=self.device)
        self.sub_cqi = torch.full((self.n_ue,self.n_rbg),2,device=self.device)
        self.max_cross_corr = torch.full((self.n_ue, self.n_ue, self.n_rbg),0,device=self.device,dtype=torch.float32)
        self.mcs = torch.zeros((self.n_ue,), device=self.device, dtype=torch.long)
        self.mcs_subband = torch.zeros((self.n_ue, self.n_rbg), device=self.device, dtype=torch.long)
        self.eligible_ues = torch.full((self.n_ue,),2,device=self.device)

        self.selected_ues = torch.arange(self.max_sched_ue)
        # TCP server: wait for MATLAB to connect
        self._rxbuf = bytearray()
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((host, port))
        self._server_sock.listen(1)
        if self.verbose:
            print(f"[MatlabEnv] Listening on {host}:{port}  "
                  f"obs_dim={self.obs_dim}  act_dim={self.act_dim}  "
                  f"n_layers={self.n_layers}  n_rbg={self.n_rbg}")
            print("[MatlabEnv] Waiting for MATLAB to connect...")
        self._client_sock, addr = self._server_sock.accept()
        if self.verbose:
            print(f"[MatlabEnv] MATLAB connected from {addr}")

    def reset(self):
        """Reset internal state (call once before the training loop)."""
        self._alloc.fill_(self.noop)
        self._cur_layer = None
        self._last_metrics = {}

    def _build_masks(self, layer: Optional[int] = None) -> torch.Tensor:
        if layer is None:
            layer = self._cur_layer if self._cur_layer is not None else 0

        # valid UE if buffer > 0, NOOP always valid
        valid_ue = (self.selected_buf > 0.0).unsqueeze(0).expand(self.n_rbg, -1)  # [M, U]

        # Rank constraint: UE rank >= total allocated layers for this RBG
        # We check: count(allocs in previous layers) < ue_rank
        # Note: This counts spatial layers per RBG. Allocating multiple RBGs on the same layer
        #       does NOT increase the rank count (it counts as 1 layer for those RBGs).
  
        if layer > 0:
            # 1. Rank Check (Same as before)
            prev_allocs = self._alloc[:layer, :]
            u_indices = torch.arange(self.max_sched_ue, device=self.device).view(1, 1, -1)
            matches = (prev_allocs.unsqueeze(-1) == u_indices)
            counts = matches.sum(dim=0)  # [M, U]
            rank_ok = (counts < self.selected_rank.unsqueeze(0)) 

            # 2. Per-UE Continuity Check. Has the UE ever been seen in this RBG before?
            ever_seen = counts > 0  # [M, U]
            
            # Was the UE in the layer immediately before this one?
            last_layer_alloc = self._alloc[layer - 1, :]
            in_prev_layer = (last_layer_alloc.unsqueeze(-1) == torch.arange(self.max_sched_ue, device=self.device).view(1, -1)) # [M, U]

            # Logic: If ever_seen is True, then in_prev_layer MUST be True.
            # If ever_seen is False, the UE is a "new starter" and is valid.
            continuity_ok = (~ever_seen) | in_prev_layer
            
            valid_ue = valid_ue & rank_ok & continuity_ok

        masks = valid_ue
        noop_col = torch.ones((self.n_rbg, 1), device=self.device, dtype=torch.bool)
        return torch.cat([masks, noop_col], dim=1)  # [M, A]

    def _build_obs(self, layer: int) -> torch.Tensor:
        # Build structured features then pad/truncate to obs_dim.

        # 1. Normalized Past Averaged Throughput [1]: avg_tp_u / max(avg_tp)
        norm_past_avg_tp = self.selected_avg_tp / self.max_rate

        # 2. Normalized Rank of UE [1]: rank_u / max_ue_rank
        norm_ue_rank = torch.clamp(self.selected_rank / float(self.max_ue_rank), 0.0, 1.0)

        # 3. Normalized Number of Already Allocated RBGs [1] (layers < current)
        if layer > 0:
            prev_alloc = self._alloc[:layer, :]  # [L', M]
            alloc_counts = torch.zeros((self.max_sched_ue,), device=self.device, dtype=torch.float32)
            for u in range(self.max_sched_ue):
                alloc_counts[u] = (prev_alloc == u).sum()
            norm_allocated_rbgs = alloc_counts / float(max(self.n_rbg, 1))
        else:
            norm_allocated_rbgs = torch.zeros((self.max_sched_ue,), device=self.device, dtype=torch.float32)

        # 4. Normalized Downlink Buffer Status [1]: buf_u / max(buf)
        max_buf = torch.clamp(self.selected_buf.max(), min=self.eps)
        norm_buffer = self.selected_buf / max_buf

        # 5. Normalized Wideband (CQI->MCS) [1]: mcs_u / 28
        mcs = torch.tensor(self.selected_curr_mcs , device=self.device, dtype=torch.float32)
        norm_wb_cqi = torch.clamp(mcs / float(self.max_mcs), 0.0, 1.0)

        # 6. Normalized Subband CQI    
        norm_subband_cqi = torch.tensor(self.mcs_subband[self.selected_ues,:], device=self.device, dtype=torch.float32) / float(self.max_mcs)

        # 7. Max Cross Correlation with previously scheduled UEs on the same RBG, Shape: [n_ue, n_rbg]
        max_corr_feat = torch.zeros((self.max_sched_ue, self.n_rbg), device=self.device, dtype=torch.float32)
        if layer > 0:
            # Convert table to tensor (shape [U, U, M])
            cross_corr = torch.as_tensor(self.max_cross_corr, device=self.device, dtype=torch.float32)
            prev_alloc = self._alloc[:layer, :]  # [L', M]
            candidate_global_ids = self.selected_ues

            for m in range(self.n_rbg):
                # Identify UEs scheduled in this RBG in previous layers
                scheduled_ues_local = prev_alloc[:, m]
                valid_mask = (scheduled_ues_local != self.noop)
                valid_ues_local = scheduled_ues_local[valid_mask]  # [k] local indices

                if valid_ues_local.numel() > 0:
                    # Map local indices to global indices
                    scheduled_ues_global = self.selected_ues[valid_ues_local]

                    # Get correlations between all candidates and already-scheduled UEs
                    sub_corr_matrix = cross_corr[candidate_global_ids.unsqueeze(1), scheduled_ues_global.unsqueeze(0), m]
                    vals, _ = sub_corr_matrix.max(dim=1)
                    max_corr_feat[:, m] = vals

        ue_feats = torch.stack(
            [norm_past_avg_tp, norm_ue_rank, norm_allocated_rbgs, norm_buffer, norm_wb_cqi],
            dim=1,
        )
        
        if(self.internal_log):
          print("UE Feats:", ue_feats)

        ue_feats = torch.cat([ue_feats, norm_subband_cqi, max_corr_feat], dim=1)  # [U, 5 + 2*n_rbg]
        core = ue_feats.reshape(-1).float()

        if core.numel() >= self.obs_dim:
            return core[: self.obs_dim].clone()
        out = torch.zeros((self.obs_dim,), device=self.device, dtype=torch.float32)
        out[: core.numel()] = core
        return out

    def begin_tti(self):

        # Resets per-TTI allocation cache.
        self._alloc.fill_(self.noop)
        self._cur_layer = None

        # Wait for TTI_START from MATLAB.
        msg = self._recv()
        msg_type= msg.get("type")
        if msg_type == "STOP":
            return 0
        if msg_type != "TTI_START":
            got = msg.get("type")
            hint = ""
            if got == "TTI_OBS":
                hint = (
                    " MATLAB is using the old protocol. "
                    "Set drlScheduler.TrainingMode = true; in MU_MIMO.m before connecting."
                )
            raise RuntimeError(
                f"[MatlabEnv] Expected TTI_START, got: {got}.{hint}"
            )
        # Validate n_layers / n_rbg if MATLAB sends them (first TTI)
        if "n_layers" in msg and msg["n_layers"] != self.n_layers:
            raise RuntimeError(
                f"[MatlabEnv] n_layers mismatch: MATLAB={msg['n_layers']}, "
                f"Python={self.n_layers}. Pass --n_layers {msg['n_layers']} to train_matlab.py."
            )
        if "n_rbg" in msg and msg["n_rbg"] != self.n_rbg:
            raise RuntimeError(
                f"[MatlabEnv] n_rbg mismatch: MATLAB={msg['n_rbg']}, "
                f"Python={self.n_rbg}. Pass --n_rbg {msg['n_rbg']} to train_matlab.py."
            )
        
        # Update UE's features from MATLAB
        self.t = msg.get("tti", "?")
        self.buf = torch.as_tensor(msg.get("buf"), device=self.device, dtype=torch.float32)
        self.inst_tp = torch.as_tensor(msg.get("avg_tp"), device=self.device, dtype=torch.float32)
        self.avg_tp = self.ema_beta * self.avg_tp + (1.0 - self.ema_beta) * self.inst_tp
        self.ue_rank = torch.as_tensor(msg.get("ue_rank"), device=self.device, dtype=torch.float32)
        self.wb_cqi = torch.as_tensor(msg.get("wb_cqi"), device=self.device, dtype=torch.long)  # n_ue
        self.sub_cqi = torch.as_tensor(msg.get("sub_cqi"), device=self.device, dtype=torch.long)  # n_ue x n_rbg
        self.max_cross_corr = torch.as_tensor(msg.get("max_cross_corr"),device=self.device,dtype=torch.float32
        )  # n_ue x n_ue x n_rbg

        self.eligible_ues = torch.as_tensor(msg.get("eligible_ues"), device=self.device, dtype=torch.long)  # n_ue
        expected_cross = (self.max_sched_ue, self.max_sched_ue, self.n_rbg)
        got_cross = tuple(self.max_cross_corr.shape)
        if got_cross != expected_cross:
            raise RuntimeError(
                f"[MatlabEnv] max_cross_corr shape mismatch: got={got_cross}, "
                f"expected={expected_cross} ([n_ue, n_ue, n_rbg])."
            )

        expected_subcqi = (self.max_sched_ue, self.n_rbg)
        got_subcqi = tuple(self.sub_cqi.shape)
        if got_subcqi != expected_subcqi:
            raise RuntimeError(
                f"[MatlabEnv] sub_cqi shape mismatch: got={got_subcqi}, "
                f"expected={expected_subcqi} ([n_ue, n_rbg])."
            )

        cqi_to_mcs = torch.as_tensor(CQI_TO_MCS, device=self.device, dtype=torch.long)
        if msg.get("curr_mcs") is not None:
            # Prefer MATLAB scheduler-computed MCS when provided.
            self.mcs = torch.as_tensor(msg.get("curr_mcs"), device=self.device, dtype=torch.long)
            self.mcs = self.mcs.clamp(0, self.max_mcs)
        else:
            # Backward-compatible fallback: derive MCS from WB CQI.
            self.mcs = cqi_to_mcs[self.wb_cqi.clamp(0, cqi_to_mcs.numel() - 1)]
        jitter = self.rng.integers(
            -self.mcs_spread,
            self.mcs_spread + 1,
            size=(self.n_ue, self.n_rbg),
        )
        sub_cqi_jittered = np.clip(
            self.sub_cqi.detach().cpu().numpy() + jitter,
            0,
            self.max_cqi,
        )
        self.sub_cqi = torch.as_tensor(sub_cqi_jittered, device=self.device, dtype=torch.long)
        self.mcs_subband = cqi_to_mcs[self.sub_cqi.clamp(0, self.max_cqi-1)]

        # UE Selection
        # selected_ues = self.ue_priority.argsort(descending=True)[:self.max_sched_ue]   
        # self.selected_ues = selected_ues
        self.selected_buf = self.buf[self.selected_ues]
        self.selected_rank = self.ue_rank[self.selected_ues]
        self.selected_avg_tp = self.avg_tp[self.selected_ues]
        self.selected_curr_mcs = self.mcs[self.selected_ues]
    
        if self.verbose:
            tti = msg.get("tti", "?")
            print(f"[MatlabEnv] === TTI {tti} ===")
        return 1

    def layer_iter(self) -> Iterator[LayerContext]:
        """
        Yield one LayerContext per spatial layer.
        Receives LAYER_OBS from MATLAB for each layer.
        """
        for l in range(self.n_layers):
            self._cur_layer = l
            masks = self._build_masks(layer=l)  # [M, A]
            obs = self._build_obs(layer=l)  # [obs_dim]
            yield LayerContext(layer=l, obs=obs, masks_rbg=masks)

    def apply_layer_actions(
        self, layer_ctx: LayerContext, actions_rbg: torch.Tensor
    ):
        """
        Extremely Important: Update actions for current layer in _alloc 
        """
        l = layer_ctx.layer
        a = actions_rbg.to(self.device).long().clamp(0, self.act_dim - 1)
        self._alloc[l, :] = a

    def compute_layer_transitions(self,  layer_ctx: LayerContext) -> List[Dict]:
        layer = layer_ctx.layer
        obs = layer_ctx.obs
        masks = layer_ctx.masks_rbg

        # Calculating reward for current layer
        rewards_m, _ = self.new_reward_compute(layer, masks)
        
        # Generating next observations
        next_layer_idx = min(layer + 1, self.n_layers - 1)
        next_masks = self._build_masks(layer=next_layer_idx)
        next_obs = self._build_obs(layer=next_layer_idx)

        # if(self.verbose):
        #   print("Next obesrvation for layer :",next_layer_idx)
        #   print(next_masks)
        #   print(next_obs)

        out = []
        for m in range(self.n_rbg):
            out.append({
                "observation": obs.detach().cpu(),
                "next_observation": next_obs.detach().cpu(),
                "rbg_index": torch.tensor(m, dtype=torch.long),
                "action": torch.tensor(int(self._alloc[layer, m].item()), dtype=torch.long),
                "reward": torch.tensor(float(rewards_m[m].item()), dtype=torch.float32),
                "action_mask": masks[m].detach().cpu(),           # [A] bool
                "next_action_mask": next_masks[m].detach().cpu(), # [A] bool
            })
        return out
    
    def finish_tti(self):

        # Update ue selection priority, reset to 0 if allocated, increase 1 otherwise
        self.ue_priority +=1
        allocated_ue=[]
        for u in range(self.max_sched_ue):
            ue_id = self.selected_ues[u]
            if (u==self._alloc).any():
                self.ue_priority[ue_id]=0
                allocated_ue.append(int(ue_id))
        unscheduled_mask = torch.full((self.n_ue,),True, device=self.device, dtype=torch.bool)
        unscheduled_mask[allocated_ue] = False

        # Guardrail: local action -> RNTI mapping must remain within MATLAB eligible set.
        self._assert_no_noneligible_actions()

        # Send allocation results of a whole TTI to MATLAB
        a = self._alloc.cpu().tolist()
        self._send({"type": "LAYER_ACT", "actions": a})

        # Wait for TTI_DONE from MATLAB
        msg = self._recv()
        if msg.get("type") != "TTI_DONE":
            raise RuntimeError(
                f"[MatlabEnv] Expected TTI_DONE, got: {msg.get('type')}"
            )
        self._last_metrics = msg.get("metrics", {})

    def _assert_no_noneligible_actions(self) -> None:
        """Fail fast if Python schedules UEs outside MATLAB's eligible set."""
        eligible_list = self.eligible_ues.detach().cpu().tolist()
        eligible_set = {int(rnti) for rnti in eligible_list if int(rnti) > 0}
        alloc = self._alloc.detach().cpu()

        bad = []
        for l in range(self.n_layers):
            for m in range(self.n_rbg):
                a = int(alloc[l, m].item())
                if a == self.noop:
                    continue
                if a < 0 or a >= self.max_sched_ue:
                    bad.append((l, m, a, -1))
                    continue
                rnti = int(self.selected_ues[a]) + 1
                if rnti not in eligible_set:
                    bad.append((l, m, a, rnti))

        if not bad:
            return

        sample = ", ".join(
            f"(l={l},m={m},a={a},rnti={rnti})" for (l, m, a, rnti) in bad[:12]
        )
        msg = (
            f"[MatlabEnv][TTI {self.t}] Non-eligible UE scheduled in Python actions. "
            f"bad={len(bad)} eligible={sorted(eligible_set)} sample={sample}"
        )

        # Always log to make debugging easy; optionally hard-stop training.
        print(msg)
        if self.strict_eligibility_assert:
            raise AssertionError(msg)


    def dump_state(self) -> Dict:
        """Return a dict of the current environment state (buffers, rates, allocs)."""
        # Assuming 't' is your tensor (detach and cpu already called)
        t = self.max_cross_corr.detach().cpu().tolist()

        # Nested comprehension to format every float to 3 decimal places
        clean_list = [[[f"{val:.1f}" for val in row] for row in depth] for depth in t]

        return {
            "t": self.t,
            "n_ue":self.n_ue,
            "eligible_ues":self.eligible_ues,
            "max_sched_ue":self.max_sched_ue,
            "buf": self.buf.detach().cpu().tolist(),
            "inst_tp":self.inst_tp.detach().cpu().tolist(),
            "avg_tp": self.avg_tp.detach().cpu().tolist(),
            "alloc": self._alloc.detach().cpu().tolist(),
            "curr_mcs": self.mcs.detach().cpu().tolist(),
            "ue_rank": self.ue_rank.detach().cpu().tolist(),
            "sub_cqi": self.sub_cqi.detach().cpu().tolist(),
            "wb_cqi": self.wb_cqi.detach().cpu().tolist(),
            "max_cross_corr": clean_list,
            "cur_layer": self._cur_layer,
            "metrics": dict(self._last_metrics)
        }
    
    def compute_set_tput(self, alloc_set, m):
        if len(alloc_set) == 0:
            return 0.0

        max_corr = 0.0
        for i in range(len(alloc_set) - 1):
            u = alloc_set[i]
            for j in range(i + 1, len(alloc_set)):
                v = alloc_set[j]
                max_corr = max(
                    max_corr,
                    float(self.max_cross_corr[self.selected_ues[u], self.selected_ues[v], m].item())
                )

        penalty = (1.0 - max_corr) / max(len(alloc_set), 1)
        # penalty = (1.0 - max_corr)

        tput = 0.0
        for u in alloc_set:
            if self.selected_buf[u] > 0:
                tput += float(self._rate_mbps(self.selected_ues[u], m).item())

        return tput * penalty

    def new_reward_compute(self, layer: int, masks: torch.Tensor):
        rewards_m = torch.zeros((self.n_rbg,), device=self.device)
        set_tp_per_rbg = torch.zeros((self.n_rbg,), device=self.device)
        noop = self.noop

        Ru_all = self.selected_avg_tp.clone()

        for m in range(self.n_rbg):
            # ---------- previous set ----------
            prev_alloc = []
            if layer > 0:
                for l_prev in range(layer):
                    u_prev = int(self._alloc[l_prev, m].item())
                    if u_prev != noop:
                        prev_alloc.append(u_prev)

            T_prev = self.compute_set_tput(prev_alloc, m)

            # ---------- compute raw rewards for all candidates ----------
            chosen = int(self._alloc[layer, m].item())
            if chosen == noop:
                set_tp_per_rbg[m] = T_prev

            raw_all = torch.zeros((self.max_sched_ue,), device=self.device)

            for u in range(self.max_sched_ue):
                if not masks[m, u] or self.selected_buf[u] <= 0:
                    continue

                curr_alloc = prev_alloc + [u]
                T_cur = self.compute_set_tput(curr_alloc, m)

                if u == chosen:
                    set_tp_per_rbg[m] = T_cur

                raw_all[u] = (T_cur - T_prev) / float(Ru_all[u].item())

            # ---------- normalization (Eq. 21) ----------
            max_raw = torch.max(raw_all)

            if max_raw > 0.0:
                if chosen == noop:
                    rewards_m[m] = 0.0
                else:
                    rewards_m[m] = torch.clamp(raw_all[chosen] / max_raw, -1.0, 1.0)
            elif max_raw < 0.0:
                # all UE choices reduce throughput → noop is optimal
                rewards_m[m] = 1.0 if chosen == noop else -1.0
            else:
                rewards_m[m] = 0.0

            if self.internal_log:
                print(f"Computing reward for rgb {m} ....")
                print(f"prev_alloc={prev_alloc}, T_prev={T_prev}")
                print(f"chosen={chosen}, T_cur={set_tp_per_rbg[m]}")
                print(f"raw_all={raw_all}")
                print(f"rewards_m={rewards_m}")
        return rewards_m, set_tp_per_rbg

    def _served_bytes(self, ue: int, m: int) -> torch.Tensor:
        mcs = int(self.mcs_subband[ue,m])
        tbs = tbs_38214_bytes(mcs, self.prbs_per_rbg, n_symb=self.n_symb, overhead_re_per_prb=self.overhead)
        return torch.tensor(float(tbs), device=self.device)

    def _rate_mbps(self, ue: int, m: int) -> torch.Tensor:
        served = self._served_bytes(ue, m)
        duration_s = self.tti_ms / 1000.0
        return (served * 8.0) / 1e6 / max(duration_s, 1e-9)
    
    def _recv(self) -> dict:
        return _recv_json(self._client_sock, self._rxbuf)

    def _send(self, obj: dict) -> None:
        _send_json(self._client_sock, obj)

    def close(self):
        """Close TCP connection."""
        try:
            self._client_sock.close()
        except Exception:
            pass
        try:
            self._server_sock.close()
        except Exception:
            pass
        if self.verbose:
            print("[MatlabEnv] Connection closed.")
