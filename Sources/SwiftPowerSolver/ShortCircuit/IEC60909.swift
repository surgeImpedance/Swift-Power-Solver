import Foundation

// IEC 60909 short-circuit calculation — initial symmetrical short-circuit
// current and the peak / thermal-equivalent currents derived from it, for
// bus faults.
//
//   Ik'' = c · Un / (√3 · |Zk|)      initial symmetrical short-circuit current
//   Zk   = Zbus[f, f]                Thévenin impedance at the fault bus, from
//                                    (Ybus + source shunts)⁻¹
//   ip   = κ · √2 · Ik''             peak current; κ via IEC method C
//   Ith  = Ik'' · √(m + n)           thermal-equivalent current
//
// The positive-sequence Ybus is the branch series admittances (line charging
// and loads are neglected, per IEC 60909) plus, at each source bus, a shunt
// 1/(c·Zsource) — the external-grid equivalent. Zbus is that inverted; only
// the diagonal is needed for bus faults, computed with the multi-RHS sparse
// solve (a complex system solved via a real 2n×2n embedding, reusing
// SparseLinearSolver — the single-RHS path is untouched).
//
// SCOPE (Piece 4): three-phase and two-phase (line-to-line) faults, with
// external-grid-type sources and lines. Deliberately NOT built — plug points:
//   // ZERO SEQUENCE        line-to-ground / L-L-G faults: zero-sequence
//                           network (r0/x0 branches, transformer vector groups).
//   // GENERATOR CORRECTION IEC impedance correction factors K_G / K_T / K_S
//                           for machine and transformer sources.
//   // IB IK                breaking (Ib) and steady-state (Ik) currents:
//                           need machine decay data (μ, q factors).
//   // BRANCH FAULT         mid-line / branch fault locations (buses only here).
//   // FAULT SIMULATION     applying a fault and re-solving the AC power flow
//                           (a different, non-IEC calculation).
//   // PROTECTION           relay logic / breaker tripping / coordination.

public struct ShortCircuitOptions: Sendable {
    public enum FaultType: Sendable { case threePhase, twoPhase }

    /// Voltage factor c (IEC 60909 Table 1). 1.1 = c_max for HV; the caller
    /// selects c per the network's nominal voltage and max/min case.
    public var voltageFactorC: Double
    public var faultType: FaultType
    /// System frequency, Hz — used by the peak (κ) and thermal (m) factors.
    public var systemFrequencyHz: Double
    /// Fault duration Tk, seconds — the thermal-equivalent current's window.
    public var faultDurationS: Double
    public var computePeak: Bool
    public var computeThermal: Bool

    public init(voltageFactorC: Double = 1.1,
                faultType: FaultType = .threePhase,
                systemFrequencyHz: Double = 50,
                faultDurationS: Double = 1.0,
                computePeak: Bool = true,
                computeThermal: Bool = true) {
        self.voltageFactorC = voltageFactorC
        self.faultType = faultType
        self.systemFrequencyHz = systemFrequencyHz
        self.faultDurationS = faultDurationS
        self.computePeak = computePeak
        self.computeThermal = computeThermal
    }
}

public struct BusFaultResult: Sendable {
    public enum Outcome: Equatable, Sendable {
        /// Fault currents were computed.
        case solved
        /// Bus is marked isolated in the network.
        case isolatedBus
        /// The bus is in a component with no fault-current source (no
        /// generator with a subtransient impedance), so no current flows.
        case noSourceFeeding
    }

    public let bus: Int
    public let outcome: Outcome
    /// Initial symmetrical short-circuit current, kA. NaN unless `.solved`.
    public let ikssKa: Double
    /// Short-circuit power S″k = √3·Un·Ik″, MVA.
    public let skssMva: Double
    /// Peak current, kA. NaN unless computed.
    public let ipKa: Double
    /// Thermal-equivalent current, kA. NaN unless computed.
    public let ithKa: Double
    /// Fault-point impedance, ohm (Zbus[f,f] referred to the bus's base kV).
    public let rkOhm: Double
    public let xkOhm: Double

    static func unsolved(_ bus: Int, _ outcome: Outcome) -> BusFaultResult {
        BusFaultResult(bus: bus, outcome: outcome, ikssKa: .nan, skssMva: .nan,
                       ipKa: .nan, ithKa: .nan, rkOhm: .nan, xkOhm: .nan)
    }
}

public struct ShortCircuitResult: Sendable {
    public let faultType: ShortCircuitOptions.FaultType
    /// One result per bus, parallel to `network.buses`.
    public let buses: [BusFaultResult]
}

public struct ShortCircuitAnalyzer {

    public init() {}

    /// Equivalent-frequency ratio for IEC 60909 method C (fc/f = 0.4 at both
    /// 50 and 60 Hz): κ is evaluated from the network R/X at this frequency.
    private static let equivalentFrequencyRatio = 0.4

    public func faults(_ net: BusBranchNetwork,
                       options: ShortCircuitOptions = ShortCircuitOptions())
        -> ShortCircuitResult {
        let n = net.busCount
        let c = options.voltageFactorC

        // Fault-current sources: generators with a subtransient impedance.
        var sourceBus = [Bool](repeating: false, count: n)
        for g in net.generators where g.inService {
            if g.scSubtransientRPu != nil || g.scSubtransientXPu != nil {
                sourceBus[g.bus] = true
            }
        }

        // A bus can be faulted if it is not isolated and its component (over
        // in-service branches) contains a source. Union-find over branches.
        var parent = Array(0..<n)
        func find(_ x: Int) -> Int {
            var r = x
            while parent[r] != r { parent[r] = parent[parent[r]]; r = parent[r] }
            return r
        }
        func union(_ a: Int, _ b: Int) { parent[find(a)] = find(b) }
        for br in net.branches where br.inService {
            if net.buses[br.from].type != .isolated,
               net.buses[br.to].type != .isolated {
                union(br.from, br.to)
            }
        }
        var componentHasSource = [Int: Bool]()
        for i in 0..<n where sourceBus[i] && net.buses[i].type != .isolated {
            componentHasSource[find(i), default: false] = true
        }

        var fed = [Bool](repeating: false, count: n)
        for i in 0..<n where net.buses[i].type != .isolated {
            fed[i] = componentHasSource[find(i)] ?? false
        }

        // Zbus diagonals at the full frequency (magnitude) and at the
        // equivalent frequency (κ's R/X), over the fed buses only.
        let zk = zbusDiagonal(net, fed: fed, sourceBus: sourceBus, c: c,
                              reactanceScale: 1.0)
        let zc = options.computePeak
            ? zbusDiagonal(net, fed: fed, sourceBus: sourceBus, c: c,
                           reactanceScale: Self.equivalentFrequencyRatio)
            : nil

        // 2-phase current is √3/2 of the 3-phase value (Z2 = Z1 for static
        // equipment and external grids).
        let faultScale = options.faultType == .twoPhase ? sqrt(3) / 2 : 1.0

        var results: [BusFaultResult] = []
        results.reserveCapacity(n)
        for i in 0..<n {
            if net.buses[i].type == .isolated {
                results.append(.unsolved(i, .isolatedBus)); continue
            }
            guard fed[i], let zki = zk?[i] else {
                results.append(.unsolved(i, .noSourceFeeding)); continue
            }

            let baseKv = net.buses[i].baseKv
            let iBaseKa = net.baseMVA / (sqrt(3) * baseKv)     // system base current
            let zBaseOhm = baseKv * baseKv / net.baseMVA
            let magZ = zki.magnitude
            let ikssPu = c / magZ                              // prefault V = c pu
            let ikssKa = ikssPu * iBaseKa * faultScale
            // S″k = √3·Un·I″k is a three-phase quantity; it is not defined the
            // same way for a two-phase fault, so report it for 3-phase only.
            let skssMva = options.faultType == .threePhase
                ? c / magZ * net.baseMVA : .nan

            var ipKa = Double.nan
            var ithKa = Double.nan
            if let zc, let zci = zc[i] {
                let kappa = Self.kappa(rOverX: (zci.re / zci.im)
                                        * Self.equivalentFrequencyRatio)
                if options.computePeak {
                    ipKa = kappa * sqrt(2) * ikssKa
                }
                if options.computeThermal {
                    let m = Self.thermalM(kappa: kappa,
                                          f: options.systemFrequencyHz,
                                          tk: options.faultDurationS)
                    // Far-from-generator fault: Ik = Ik'' so n = 1.
                    ithKa = ikssKa * (m + 1.0).squareRoot()
                }
            }

            results.append(BusFaultResult(
                bus: i, outcome: .solved, ikssKa: ikssKa, skssMva: skssMva,
                ipKa: ipKa, ithKa: ithKa,
                rkOhm: zki.re * zBaseOhm, xkOhm: zki.im * zBaseOhm))
        }

        return ShortCircuitResult(faultType: options.faultType, buses: results)
    }

    // MARK: - Zbus diagonal

    /// Diagonal of Zbus = (Ybus + source shunts)⁻¹, in pu, for the fed buses;
    /// `nil` entries for non-fed buses. Reactances are scaled by
    /// `reactanceScale` (1.0 for the fault magnitude, fc/f for κ's R/X).
    private func zbusDiagonal(_ net: BusBranchNetwork, fed: [Bool],
                              sourceBus: [Bool], c: Double,
                              reactanceScale s: Double) -> [ComplexD?]? {
        let n = net.busCount
        var local = [Int](repeating: -1, count: n)
        var fedBuses: [Int] = []
        for i in 0..<n where fed[i] { local[i] = fedBuses.count; fedBuses.append(i) }
        let m = fedBuses.count
        guard m > 0 else { return [ComplexD?](repeating: nil, count: n) }

        // Complex Ybus over fed buses: branch series + source shunts.
        var y = [ComplexD](repeating: .zero, count: m * m)   // row-major dense
        func add(_ r: Int, _ cc: Int, _ v: ComplexD) { y[r * m + cc] += v }
        for br in net.branches where br.inService {
            guard fed[br.from], fed[br.to] else { continue }
            let ys = ComplexD(1, 0) / ComplexD(br.r, br.x * s)
            let f = local[br.from], t = local[br.to]
            add(f, f, ys); add(t, t, ys); add(f, t, -ys); add(t, f, -ys)
        }
        for g in net.generators where g.inService {
            guard sourceBus[g.bus], fed[g.bus] else { continue }
            let zsrc = c * ComplexD(g.scSubtransientRPu ?? 0,
                                    (g.scSubtransientXPu ?? 0) * s)
            add(local[g.bus], local[g.bus], ComplexD(1, 0) / zsrc)
        }

        // Solve Y·X = I via the real 2m×2m embedding [[G,-B],[B,G]]; one RHS
        // per fed bus gives that Zbus column, whose diagonal entry we keep.
        var entries: [(row: Int, col: Int, value: Double)] = []
        entries.reserveCapacity(m * m * 2)
        for r in 0..<m {
            for cc in 0..<m {
                let v = y[r * m + cc]
                guard v.re != 0 || v.im != 0 else { continue }
                entries.append((r, cc, v.re))              // G
                entries.append((r, m + cc, -v.im))         // -B
                entries.append((m + r, cc, v.im))          // B
                entries.append((m + r, m + cc, v.re))      // G
            }
        }
        var rhs: [[Double]] = []
        rhs.reserveCapacity(m)
        for i in 0..<m {
            var e = [Double](repeating: 0, count: 2 * m)
            e[i] = 1                                        // real unit injection
            rhs.append(e)
        }
        guard let sol = SparseLinearSolver.solve(n: 2 * m, entries: entries,
                                                 rhsColumns: rhs) else {
            return nil
        }

        var out = [ComplexD?](repeating: nil, count: n)
        for (i, bus) in fedBuses.enumerated() {
            out[bus] = ComplexD(sol[i][i], sol[i][m + i])   // Zbus[bus,bus]
        }
        return out
    }

    // MARK: - IEC factors

    /// κ for the peak current (IEC 60909 method C): κ = 1.02 + 0.98·e^(−3·R/X),
    /// with R/X taken at the equivalent frequency by the caller.
    static func kappa(rOverX: Double) -> Double {
        1.02 + 0.98 * exp(-3 * rOverX)
    }

    /// DC (aperiodic) thermal factor m (IEC 60909-0):
    ///   m = (e^{4·ln(κ−1)·f·Tk} − 1) / (2·f·Tk·ln(κ−1)),  m → 2 as κ → 2.
    static func thermalM(kappa: Double, f: Double, tk: Double) -> Double {
        let l = log(kappa - 1)
        if abs(l) < 1e-9 { return 2.0 }                    // κ → 2 limit
        return (exp(4 * l * f * tk) - 1) / (2 * f * tk * l)
    }
}
