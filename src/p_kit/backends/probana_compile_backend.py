"""
Probana protocol version 2.
Compiles large p-kit circuits into banks and supports full annealing.
Matching firmware must be used.
Banking can be slow and in some respects sub-optimal, but it allows testing
with many p-bits while still mapping them onto physical p-bits, which gives
us a good estimate of how a future version of Probana with many physical p-bits
will behave.

This module keeps circuit analysis on the host and sends an execution-ready
bank plan to the Probana firmware. A logical circuit may therefore contain
many more p-bits than the physical device. The firmware only has to execute
the precomputed banks and their bank-major sparse incoming-edge lists.

Bank-update modes define when each bank’s new states become visible to other
banks, ensuring that banking preserves the intended p-bit update dynamics.
Two bank-update modes are supported:
  * snapshot:
    Every bank reads the same old logical state and writes a separate new
    state. The new state is committed after all banks have executed. This
    matches a synchronous/vector update and always needs exactly
    ceil(n_virtual / n_physical) bank executions.
  * sequential:
    A bank is committed before the next bank executes. Nodes placed in the
    same bank must therefore be independent in the coupling graph. The host
    compiler solves a capacity-constrained graph-colouring problem, using an
    exact MILP when the greedy schedule does not already meet the lower bound.

Banking strategies determine how virtual p-bits are grouped and mapped onto 
the available physical p-bits. 4 modes:
  * linear:   groups virtual p-bits consecutively without analysing circuit structure.
  * balanced: distributes p-bits to balance the coupling-computation load across banks.
  * graph:    groups mutually uncoupled p-bits so they can be updated safely in parallel.
  * auto:     automatically selects balanced for snapshot updates and graph for sequential updates.

Protocol version 2 extends the original Probana text protocol with::

    VCLEAR <n_virtual> <n_physical> <n_banks> <nnz> <mode>
    VBANK <bank> <count> <virtual_0> ... <virtual_count-1>
    H <virtual> <value>
    J <source> <destination> <value>
    VCOMMIT
    ANNEAL TABLE <count>
    ANNEAL VALUES <start> <count> <value_0> ... <value_count-1>
    ANNEAL COMMIT

``J[source, destination]`` follows p-kit's ``m @ J`` convention. J commands
are deliberately emitted in bank/node/incoming-edge order so constrained
firmware can retain the received order as its execution representation.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
from typing import Iterable

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp
from scipy.sparse import coo_matrix

from .numpy_backend import NumpyBackend
from .probana_backend import ProbanaBackend

_UPDATE_MODES = {"snapshot", "sequential"}
_BANKING_POLICIES = {"auto", "linear", "balanced", "graph"}

@dataclass(frozen=True)
class ProbanaExecutionPlan:
    version: int
    n_virtual: int
    n_physical: int
    update_mode: str
    banking_policy: str
    optimal_bank_count: bool
    bank_ptr: tuple[int, ...]
    nodes: tuple[int, ...]
    h: tuple[float, ...]
    edge_ptr: tuple[int, ...]
    edge_src: tuple[int, ...]
    edge_weight: tuple[float, ...]

    @property
    def n_banks(self):
        return len(self.bank_ptr) - 1

    @property
    def nnz(self):
        return len(self.edge_src)

    @property
    def banks(self):
        return tuple(self.nodes[self.bank_ptr[i]:self.bank_ptr[i + 1]]
                     for i in range(self.n_banks))

    def incoming(self, scheduled_position):
        start, end = self.edge_ptr[scheduled_position:scheduled_position + 2]
        return tuple(zip(self.edge_src[start:end], self.edge_weight[start:end]))

    def validate(self):
        if self.version != 2:
            raise ValueError("unsupported execution-plan version")
        if self.update_mode not in _UPDATE_MODES:
            raise ValueError("invalid update mode")
        if self.banking_policy not in _BANKING_POLICIES - {"auto"}:
            raise ValueError("invalid resolved banking policy")
        if self.n_virtual < 1 or self.n_physical < 1:
            raise ValueError("p-bit counts must be positive")
        if len(self.h) != self.n_virtual:
            raise ValueError("h length does not match n_virtual")
        if not self.bank_ptr or self.bank_ptr[0] != 0 or self.bank_ptr[-1] != len(self.nodes):
            raise ValueError("bank_ptr does not cover nodes")

        sizes = [b - a for a, b in zip(self.bank_ptr, self.bank_ptr[1:])]
        if any(size < 1 or size > self.n_physical for size in sizes):
            raise ValueError("invalid bank size")
        if sorted(self.nodes) != list(range(self.n_virtual)):
            raise ValueError("each virtual p-bit must occur exactly once")
        if len(self.edge_ptr) != len(self.nodes) + 1 or not self.edge_ptr or self.edge_ptr[0] != 0:
            raise ValueError("edge_ptr does not match nodes")
        if any(a > b for a, b in zip(self.edge_ptr, self.edge_ptr[1:])) or self.edge_ptr[-1] != self.nnz:
            raise ValueError("edge_ptr does not cover edges")
        if len(self.edge_weight) != self.nnz:
            raise ValueError("edge arrays have different lengths")
        if any(src < 0 or src >= self.n_virtual for src in self.edge_src):
            raise ValueError("edge source is outside the virtual circuit")
        if not np.all(np.isfinite(self.h)) or not np.all(np.isfinite(self.edge_weight)):
            raise ValueError("plan contains a non-finite value")

        if self.update_mode == "sequential":
            for bank_id, bank in enumerate(self.banks):
                members, start = set(bank), self.bank_ptr[bank_id]
                for position in range(start, self.bank_ptr[bank_id + 1]):
                    dst = self.nodes[position]
                    if any(src != dst and src in members
                           for src, _ in self.incoming(position)):
                        raise ValueError("a sequential bank contains coupled nodes")
        return self

    def to_dict(self):
        return {
            "version": self.version, "n_virtual": self.n_virtual,
            "n_physical": self.n_physical, "update_mode": self.update_mode,
            "banking_policy": self.banking_policy,
            "optimal_bank_count": self.optimal_bank_count,
            "bank_ptr": list(self.bank_ptr), "nodes": list(self.nodes),
            "h": list(self.h), "edge_ptr": list(self.edge_ptr),
            "edge_src": list(self.edge_src),
            "edge_weight": list(self.edge_weight)
        }

    def to_json(self, **kwargs):
        return json.dumps(self.to_dict(), **kwargs)

def _validate_circuit(J, h):
    J, h = np.asarray(J, dtype=float), np.asarray(h, dtype=float).reshape(-1)
    if J.ndim != 2 or J.shape[0] != J.shape[1]:
        raise ValueError("J must be square")
    if J.shape[0] != h.size:
        raise ValueError("J and h sizes do not match")
    if not h.size:
        raise ValueError("the circuit must contain at least one p-bit")
    if not np.all(np.isfinite(J)) or not np.all(np.isfinite(h)):
        raise ValueError("J and h must contain only finite values")
    return J, h

def _conflict_graph(J, tolerance):
    present = np.abs(J) > tolerance
    conflict = present | present.T
    np.fill_diagonal(conflict, False)
    return conflict

def _linear_banks(n, capacity):
    return [list(range(i, min(i + capacity, n)))
            for i in range(0, n, capacity)]

def _balanced_banks(load, capacity):
    banks = [[] for _ in range(math.ceil(len(load) / capacity))]
    bank_load = np.zeros(len(banks), dtype=float)

    for node in sorted(range(len(load)), key=lambda i: (-load[i], i)):
        candidates = [b for b in range(len(banks))
                      if len(banks[b]) < capacity]
        bank = min(candidates,
                   key=lambda b: (bank_load[b], len(banks[b]), b))
        banks[bank].append(node)
        bank_load[bank] += load[node]
    return banks

def _compatible(node, bank, conflict):
    return not bank or not np.any(conflict[node, bank])

def _greedy_graph_banks(conflict, capacity, load):
    degree = np.count_nonzero(conflict, axis=1)
    rng = np.random.default_rng(0)
    orders = [
        sorted(range(len(load)), key=lambda i: (-degree[i], -load[i], i)),
        sorted(range(len(load)), key=lambda i: (-load[i], -degree[i], i))
    ]

    for _ in range(14):
        jitter = rng.random(len(load))
        orders.append(sorted(range(len(load)),
                             key=lambda i: (-degree[i], jitter[i])))

    best = None
    for order in orders:
        banks, bank_load = [], []
        for node in order:
            candidates = [
                b for b, bank in enumerate(banks)
                if len(bank) < capacity and _compatible(node, bank, conflict)
            ]
            if candidates:
                bank = min(candidates,
                           key=lambda b: (-len(banks[b]), bank_load[b], b))
                banks[bank].append(node)
                bank_load[bank] += load[node]
            else:
                banks.append([node])
                bank_load.append(float(load[node]))

        if best is None or len(banks) < len(best):
            best = banks
        if len(best) == math.ceil(len(load) / capacity):
            break
    return best

def _exact_graph_banks(conflict, capacity, upper, timeout):
    n = conflict.shape[0]
    lower = math.ceil(n / capacity)
    if len(upper) == lower:
        return upper, True

    edges = np.transpose(np.nonzero(np.triu(conflict, 1)))
    remaining = None if timeout is None else float(timeout)

    for n_banks in range(lower, len(upper)):
        n_vars = n * n_banks
        rows, cols, data, lb, ub, row = [], [], [], [], [], 0

        for node in range(n):
            for bank in range(n_banks):
                rows.append(row)
                cols.append(node * n_banks + bank)
                data.append(1.0)
            lb.append(1.0)
            ub.append(1.0)
            row += 1

        for bank in range(n_banks):
            for node in range(n):
                rows.append(row)
                cols.append(node * n_banks + bank)
                data.append(1.0)
            lb.append(-np.inf)
            ub.append(float(capacity))
            row += 1

        for left, right in edges:
            for bank in range(n_banks):
                rows.extend((row, row))
                cols.extend((left * n_banks + bank,
                             right * n_banks + bank))
                data.extend((1.0, 1.0))
                lb.append(-np.inf)
                ub.append(1.0)
                row += 1

        matrix = coo_matrix(
            (data, (rows, cols)), shape=(row, n_vars)
        ).tocsr()

        options = {"presolve": True}
        if remaining is not None:
            options["time_limit"] = max(0.01, remaining)

        started = time.monotonic()
        result = milp(
            c=np.zeros(n_vars),
            integrality=np.ones(n_vars),
            bounds=Bounds(0.0, 1.0),
            constraints=LinearConstraint(
                matrix, np.asarray(lb), np.asarray(ub)
            ),
            options=options
        )

        if remaining is not None:
            remaining -= time.monotonic() - started

        if result.success:
            assignment = np.rint(
                result.x.reshape(n, n_banks)
            ).astype(bool)
            return [
                np.flatnonzero(assignment[:, b]).tolist()
                for b in range(n_banks)
                if np.any(assignment[:, b])
            ], True

        if result.status != 2 or (
            remaining is not None and remaining <= 0
        ):
            return upper, False

    return upper, True

def _banks_are_independent(banks: Iterable[Iterable[int]], conflict):
    for bank in banks:
        nodes = list(bank)
        if len(nodes) > 1 and np.any(conflict[np.ix_(nodes, nodes)]):
            return False
    return True

class ProbanaCompileBackend(ProbanaBackend):
    """Compile and upload virtual p-bit banks to Probana protocol v2."""

    protocol_version = 2
    supports_pkit_annealing_func = True

    def __init__(self, port=None, baudrate=115200, timeout=2.0, startup_timeout=60.0, dtype=None, physical_pbits=None, update_mode="snapshot", banking="auto", exact=True, optimization_timeout=10.0, zero_tolerance=0.0, connect=True):
        update_mode, banking = update_mode.lower(), banking.lower()

        if update_mode not in _UPDATE_MODES:
            raise ValueError(f"unsupported update mode: {update_mode}")
        if banking not in _BANKING_POLICIES:
            raise ValueError(f"unsupported banking policy: {banking}")
        if physical_pbits is not None and int(physical_pbits) < 1:
            raise ValueError("physical_pbits must be positive")
        if optimization_timeout is not None and optimization_timeout <= 0:
            raise ValueError("optimization_timeout must be positive or None")
        if zero_tolerance < 0:
            raise ValueError("zero_tolerance must be non-negative")

        self.update_mode = update_mode
        self.banking = banking
        self.exact = bool(exact)
        self.optimization_timeout = optimization_timeout
        self.zero_tolerance = float(zero_tolerance)
        self.last_plan = None
        self.connected = bool(connect)

        if connect:
            super().__init__(
                port=port, baudrate=baudrate, timeout=timeout,
                startup_timeout=startup_timeout, dtype=dtype
            )
            if physical_pbits is not None and int(physical_pbits) != self.n_pbits:
                detected = self.n_pbits
                self.close()
                raise ValueError(
                    f"physical_pbits={physical_pbits}, "
                    f"but device reports {detected}"
                )
            self.n_physical = self.n_pbits
        else:
            if physical_pbits is None:
                raise ValueError(
                    "physical_pbits is required when connect=False"
                )
            NumpyBackend.__init__(self, dtype=dtype)
            self.port, self.timeout, self.ser = port, timeout, None
            self.n_physical = self.n_pbits = int(physical_pbits)

        self.supports_pkit_annealing_func = True

    def compile_circuit(self, J, h):
        J, h = _validate_circuit(J, h)
        n = h.size
        conflict = _conflict_graph(J, self.zero_tolerance)
        load = np.count_nonzero(
            np.abs(J) > self.zero_tolerance, axis=0
        )

        policy = self.banking
        if policy == "auto":
            policy = (
                "balanced"
                if self.update_mode == "snapshot"
                else "graph"
            )

        optimal = True
        if policy == "linear":
            banks = _linear_banks(n, self.n_physical)
        elif policy == "balanced":
            banks = _balanced_banks(load, self.n_physical)
        else:
            banks = _greedy_graph_banks(
                conflict, self.n_physical, load
            )
            optimal = len(banks) == math.ceil(
                n / self.n_physical
            )
            if self.exact and not optimal:
                banks, optimal = _exact_graph_banks(
                    conflict, self.n_physical, banks,
                    self.optimization_timeout
                )

        if (
            self.update_mode == "sequential"
            and not _banks_are_independent(banks, conflict)
        ):
            raise ValueError(
                f"banking='{policy}' places coupled nodes "
                "in one sequential bank; use banking='graph' "
                "or banking='auto'"
            )

        banks = sorted(
            (sorted(bank) for bank in banks),
            key=lambda bank: (bank[0], len(bank))
        )
        nodes = tuple(node for bank in banks for node in bank)

        bank_ptr = [0]
        for bank in banks:
            bank_ptr.append(bank_ptr[-1] + len(bank))

        edge_ptr, edge_src, edge_weight = [0], [], []
        for dst in nodes:
            sources = np.flatnonzero(
                np.abs(J[:, dst]) > self.zero_tolerance
            )
            for src in sources:
                edge_src.append(int(src))
                edge_weight.append(float(J[src, dst]))
            edge_ptr.append(len(edge_src))

        plan = ProbanaExecutionPlan(
            version=2,
            n_virtual=n,
            n_physical=self.n_physical,
            update_mode=self.update_mode,
            banking_policy=policy,
            optimal_bank_count=optimal,
            bank_ptr=tuple(bank_ptr),
            nodes=nodes,
            h=tuple(map(float, h)),
            edge_ptr=tuple(edge_ptr),
            edge_src=tuple(edge_src),
            edge_weight=tuple(edge_weight)
        ).validate()

        self.last_plan = plan
        return plan

    def set_annealing(self, mode="constant", start=1.0, end=None, steps=1):
        mode = mode.lower()
        if mode != "table":
            return super().set_annealing(mode, start, end, steps)

        values = np.asarray(start, dtype=float)
        if values.ndim != 1 or not values.size:
            raise ValueError(
                "Table annealing requires a non-empty 1D array"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError(
                "Annealing table contains a non-finite value"
            )

        self._send(f"ANNEAL TABLE {values.size}")
        self._ok()

        for offset in range(0, values.size, 8):
            chunk = values[offset:offset + 8]
            encoded = " ".join(
                f"{value:.9g}" for value in chunk
            )
            self._send(
                f"ANNEAL VALUES {offset} "
                f"{chunk.size} {encoded}"
            )
            self._ok()

        self._send("ANNEAL COMMIT")
        self._ok()

    def upload_plan(self, plan, annealing=("constant", 1.0)):
        if not self.connected or self.ser is None:
            raise RuntimeError(
                "cannot upload a plan without a connection"
            )

        plan.validate()
        if plan.n_physical != self.n_physical:
            raise ValueError(
                "plan targets a different physical p-bit count"
            )

        self._send(
            f"VCLEAR {plan.n_virtual} {plan.n_physical} "
            f"{plan.n_banks} {plan.nnz} "
            f"{plan.update_mode.upper()}"
        )
        self._ok()

        for bank_id, bank in enumerate(plan.banks):
            nodes = " ".join(map(str, bank))
            self._send(
                f"VBANK {bank_id} {len(bank)} {nodes}"
            )
            self._ok()

        for node, value in enumerate(plan.h):
            if value != 0.0:
                self._send(f"H {node} {value:.9g}")
                self._ok()

        for position, dst in enumerate(plan.nodes):
            for src, weight in plan.incoming(position):
                self._send(
                    f"J {src} {dst} {weight:.9g}"
                )
                self._ok()

        self._send("VCOMMIT")
        self._ok()
        self.set_annealing(*annealing)

    def load_circuit(self, J, h, annealing=("constant", 1.0)):
        plan = self.compile_circuit(J, h)
        self.upload_plan(plan, annealing)
        return plan