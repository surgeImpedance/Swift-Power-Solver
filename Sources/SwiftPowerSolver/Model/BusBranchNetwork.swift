import Foundation

// The reduced electrical model every solver piece operates on (AC/DC power
// flow, N-1, short-circuit). This is the package's input seam: consuming
// applications perform their own node-breaker -> bus-branch reduction (switch
// collapsing, per-unit conversion) and hand the solver this network.
//
// Everything is per-unit on `baseMVA`. Buses are dense indices 0..<n; lines
// and transformers are both `Branch` — a unified pi-model with off-nominal
// tap ratio and phase shift on the `from` side (matpower convention).

public struct BusBranchNetwork: Equatable, Sendable {
    public enum BusType: Int, Equatable, Sendable {
        case pq = 1, pv = 2, slack = 3, isolated = 4
    }

    public struct Bus: Equatable, Sendable {
        public var type: BusType
        public var baseKv: Double
        public var pLoadPu: Double   // demand
        public var qLoadPu: Double
        public var gsPu: Double      // shunt conductance at V = 1 pu
        public var bsPu: Double      // shunt susceptance at V = 1 pu (+ = capacitive)
        /// ZIP load components, per-unit at V = 1 (D80; ZIP exploration §3.1).
        /// `pLoadPu` keeps its exact meaning — the TOTAL load at V = 1,
        /// including any negative-load fold — and these are the parts of that
        /// total that are constant-impedance (Z) and constant-current (I). The
        /// remainder, `pLoadPu − pLoadZPu − pLoadIPu`, is constant power and is
        /// never stored. Absolute components rather than fractions, so a bus
        /// whose loads cancel, or that carries a folded generator, has no
        /// undefined mix. All four default to 0: every network that predates
        /// them is exactly constant power.
        ///
        /// NOTHING READS THESE YET. No solver evaluates the polynomial; the
        /// data model landed first (§5.2 step 2) and was shown inert on every
        /// fixture before any solver change existed.
        public var pLoadZPu: Double = 0
        public var pLoadIPu: Double = 0
        public var qLoadZPu: Double = 0
        public var qLoadIPu: Double = 0

        public init(type: BusType, baseKv: Double,
                    pLoadPu: Double = 0, qLoadPu: Double = 0,
                    gsPu: Double = 0, bsPu: Double = 0,
                    pLoadZPu: Double = 0, pLoadIPu: Double = 0,
                    qLoadZPu: Double = 0, qLoadIPu: Double = 0) {
            self.type = type
            self.baseKv = baseKv
            self.pLoadPu = pLoadPu
            self.qLoadPu = qLoadPu
            self.gsPu = gsPu
            self.bsPu = bsPu
            self.pLoadZPu = pLoadZPu
            self.pLoadIPu = pLoadIPu
            self.qLoadZPu = qLoadZPu
            self.qLoadIPu = qLoadIPu
        }

        /// True when any Z or I component is nonzero on this bus.
        public var hasVoltageDependentLoad: Bool {
            pLoadZPu != 0 || pLoadIPu != 0 || qLoadZPu != 0 || qLoadIPu != 0
        }
    }

    public struct Branch: Equatable, Sendable {
        public var from: Int
        public var to: Int
        public var r: Double
        public var x: Double
        public var b: Double         // total charging susceptance
        public var g: Double         // total charging conductance (trafo
                                     // magnetizing after the t->pi conversion)
        public var tap: Double       // off-nominal ratio at the from side; 1 = none
        public var shiftRad: Double  // phase shift at the from side
        public var inService: Bool
        /// Thermal rating, MVA. `nil` = unrated: contingency screening reports
        /// flows for the branch but never a violation against it.
        public var ratingMva: Double?

        public init(from: Int, to: Int, r: Double, x: Double, b: Double = 0,
                    g: Double = 0, tap: Double = 1.0, shiftRad: Double = 0,
                    inService: Bool = true, ratingMva: Double? = nil) {
            self.from = from
            self.to = to
            self.r = r
            self.x = x
            self.b = b
            self.g = g
            self.tap = tap
            self.shiftRad = shiftRad
            self.inService = inService
            self.ratingMva = ratingMva
        }
    }

    public struct Generator: Equatable, Sendable {
        public var bus: Int
        public var pPu: Double       // fixed P for PV gens; ignored for the slack
        public var vSetPu: Double    // voltage setpoint (PV and slack)
        public var vaRefRad: Double  // slack reference angle (slack gens only)
        public var qMinPu: Double
        public var qMaxPu: Double
        public var inService: Bool
        /// Positive-sequence internal (subtransient) short-circuit impedance,
        /// pu on the system base — the raw source impedance BEFORE the IEC
        /// 60909 voltage factor c is applied (e.g. Un²/S″k for an external
        /// grid, or X″d for a machine). `nil` on both ⇒ not a fault source, so
        /// the generator contributes nothing to short-circuit calculations.
        /// Used only by `ShortCircuitAnalyzer`; the power-flow solvers ignore it.
        public var scSubtransientRPu: Double?
        public var scSubtransientXPu: Double?
        /// Distributed-slack contribution weight (pandapower `slack_weight`).
        /// `nil`/0 on every generator ⇒ ordinary single-slack: the slack bus(es)
        /// absorb all imbalance, exactly as before this field existed. When any
        /// generator carries a weight, the solver switches to distributed slack —
        /// weights are normalized to sum to 1 across contributors and the power
        /// imbalance is shared proportionally. Used only by NewtonRaphsonSolver.
        public var slackWeight: Double?
        /// Active-power regulating range, pu on the system base — how far
        /// distributed slack may move this unit from `pPu`.
        ///
        /// `nil` ⇒ unbounded in that direction, the same convention `qMinPu` /
        /// `qMaxPu` use with ±infinity: an unbounded unit is never pinned, so a
        /// network with no limits set solves exactly as it did before these
        /// fields existed.
        ///
        /// Read ONLY by the distributed-slack path. Single-slack is unaffected:
        /// there the slack bus absorbs the island imbalance by definition, and
        /// clamping it would leave the network unbalanced rather than model
        /// anything physical.
        ///
        /// NOTE: pandapower does not enforce these. `runpp(distributed_slack=
        /// True)` ignores `min_p_mw` / `max_p_mw` entirely — verified against
        /// pandapower 3.2.1 both empirically and by inspection of
        /// `pypower/newtonpf.py`, which contains no P-limit code. The pinning
        /// convention below is therefore this package's own specification, and
        /// is validated against pandapower by network equivalence rather than
        /// by matching an upstream algorithm. See `DistributedSlackPLimitTests`.
        public var pMinPu: Double?
        public var pMaxPu: Double?

        public init(bus: Int, pPu: Double, vSetPu: Double, vaRefRad: Double = 0,
                    qMinPu: Double = -.infinity, qMaxPu: Double = .infinity,
                    inService: Bool = true,
                    scSubtransientRPu: Double? = nil,
                    scSubtransientXPu: Double? = nil,
                    slackWeight: Double? = nil,
                    pMinPu: Double? = nil,
                    pMaxPu: Double? = nil) {
            self.bus = bus
            self.pPu = pPu
            self.vSetPu = vSetPu
            self.vaRefRad = vaRefRad
            self.qMinPu = qMinPu
            self.qMaxPu = qMaxPu
            self.inService = inService
            self.scSubtransientRPu = scSubtransientRPu
            self.scSubtransientXPu = scSubtransientXPu
            self.slackWeight = slackWeight
            self.pMinPu = pMinPu
            self.pMaxPu = pMaxPu
        }
    }

    public var baseMVA: Double
    public var buses: [Bus]
    public var branches: [Branch]
    public var generators: [Generator]

    public init(baseMVA: Double, buses: [Bus], branches: [Branch],
                generators: [Generator]) {
        self.baseMVA = baseMVA
        self.buses = buses
        self.branches = branches
        self.generators = generators
    }

    public var busCount: Int { buses.count }

    /// True when any bus carries a nonzero Z or I load component (D80).
    ///
    /// The contract for every solver, once one reads it: evaluate this ONCE at
    /// entry and, when false, run the constant-power path UNCHANGED — not an
    /// arithmetically-equivalent new path. That is the guarantee that every
    /// network predating the components solves bit-identically. Computed
    /// rather than stored because `buses` is mutated by the sweep and a stored
    /// flag would go stale.
    public var hasVoltageDependentLoad: Bool {
        buses.contains { $0.hasVoltageDependentLoad }
    }
}
