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

        public init(type: BusType, baseKv: Double,
                    pLoadPu: Double = 0, qLoadPu: Double = 0,
                    gsPu: Double = 0, bsPu: Double = 0) {
            self.type = type
            self.baseKv = baseKv
            self.pLoadPu = pLoadPu
            self.qLoadPu = qLoadPu
            self.gsPu = gsPu
            self.bsPu = bsPu
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

        public init(bus: Int, pPu: Double, vSetPu: Double, vaRefRad: Double = 0,
                    qMinPu: Double = -.infinity, qMaxPu: Double = .infinity,
                    inService: Bool = true,
                    scSubtransientRPu: Double? = nil,
                    scSubtransientXPu: Double? = nil,
                    slackWeight: Double? = nil) {
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
}
