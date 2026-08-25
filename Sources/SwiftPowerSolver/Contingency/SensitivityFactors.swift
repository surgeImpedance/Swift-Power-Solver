import CryptoKit
import Foundation

// Public sensitivity-factor API (D64). PTDF / LODF / OTDF as queryable,
// cacheable value types over the SAME arithmetic `DistributionFactors` has
// always used — this file adds an interface, it does not add a solver.
//
// CONVENTIONS, stated here because the doc comment is the contract:
//
//   Basis         DC, lossless, active power only. `PTDFResult.basis` is
//                 `.dcLossless` and there is no other case; a consumer that
//                 reaches for these on an AC result must fail loudly.
//   Flow sign     positive from -> to.
//   PTDF          PTDF[l, b] = dF_l / dP_b, the injection at b compensated at
//                 the reference. MW per MW, dimensionless.
//   Susceptance   b_l = 1 / (x_l * tau_l), tau <= 0 coerced to 1. R ignored.
//   Phase shift   contributes an ADDITIVE term to base flow and nothing to
//                 PTDF. The term is carried on the result (see
//                 `phaseShiftBranchFlowPu`) rather than left in the DC solver,
//                 so `flows(forInjections:)` reconstructs a complete DC flow
//                 and the term is exercised by a public gate instead of going
//                 unwitnessed. No IEEE case in this repo has a phase shifter.
//   Buses         columns are ELECTRICAL buses, not substations. Busbar
//                 splitting changes the bus set, and `FactorsSignature`
//                 changes with it.

// MARK: - Identifiers

/// A bus, by its dense index in the `BusBranchNetwork` the factors were built
/// from. A wrapper rather than a bare `Int` so a bus index and a branch index
/// cannot be transposed at a call site — the network has no other identity to
/// offer, and pretending otherwise would be worse than saying so.
public struct BusID: Hashable, Sendable, Comparable, CustomStringConvertible {
    public let index: Int
    public init(_ index: Int) { self.index = index }
    public static func < (a: Self, b: Self) -> Bool { a.index < b.index }
    public var description: String { "bus \(index)" }
}

/// A branch, by its dense index in `BusBranchNetwork.branches`.
public struct BranchID: Hashable, Sendable, Comparable, CustomStringConvertible {
    public let index: Int
    public init(_ index: Int) { self.index = index }
    public static func < (a: Self, b: Self) -> Bool { a.index < b.index }
    public var description: String { "branch \(index)" }
}

/// The model the factors were computed under. One case today; it exists so a
/// future AC sensitivity cannot be silently substituted for a DC one.
public enum SensitivityBasis: String, Sendable, Hashable {
    case dcLossless
}

// MARK: - Reference scheme

/// Where the compensating injection goes (D64 §2).
public enum SlackReference: Sendable, Hashable {
    /// Every bus of type `.slack` is a reference — the behaviour N-1 has always
    /// had, and the default. Multi-reference is deliberate: it is how several
    /// islands each get a reference.
    case networkDefined
    /// Absorption spread UNIFORMLY over the given buses — which is what a
    /// post-shift of the network-defined base by uniform weights actually does.
    ///
    /// **Named for the operation, not for the intent (Q1).** An earlier spelling
    /// was `.references`, and that name asserted something the implementation
    /// does not do: a true reference set is EXCLUDED from the reduced system and
    /// absorbs per island, whereas a post-shift spreads absorption uniformly
    /// across the set regardless of island. Those are different operations —
    /// which is exactly why an island-spanning set has to be rejected here.
    ///
    /// Within one island the distinction dissolves: several *references* in a
    /// single island is over-constrained and ill-defined, so uniform
    /// distribution is the only sensible reading. Single-slack is
    /// `.uniformlyDistributed([b])`, kept as the ergonomic spelling rather than
    /// folded into `.distributed`.
    ///
    /// **Exactness:** `PTDF_networkDefined[:, s] = 0` for a reference bus `s`,
    /// so post-shifting by `s` subtracts a zero column and the single-bus case
    /// is BITWISE identical to `.networkDefined` on a single-slack network —
    /// gate 6.17. That identity (changing slack from `s` to `b` is subtraction
    /// of column `b`) is the whole justification for not opening the frozen
    /// build to support an explicit reference set.
    case uniformlyDistributed([BusID])
    /// Participation factors, normalised on construction. Computed as a
    /// post-shift of the `.networkDefined` base — the shift is base-invariant
    /// when the weights sum to 1, so there is no second factorization.
    case distributed([BusID: Double])
}

// MARK: - Errors

public enum SensitivityError: Error, Equatable, Sendable {
    case unknownBus(BusID)
    case unknownBranch(BranchID)
    case signatureMismatch(expected: FactorsSignature, actual: FactorsSignature)
    case islandingOutage(BranchID)
    case singularAdmittanceMatrix(worstResidual: Double?)
    case disconnectedNetwork
    case invalidParticipationFactors(String)
    /// A reference set that names a bus the network cannot reference.
    case invalidReference(String)
    /// A network parameter that cannot be hashed or solved with — a non-finite
    /// susceptance, reactance or tap. Distinct from
    /// `.singularAdmittanceMatrix`: that names a SOLVE that went wrong, this
    /// names an INPUT that was never valid. Conflating them would muddy the
    /// residual control D65 §4 demoted to a garbage-solve guard.
    case invalidNetworkParameter(String)
}

// MARK: - Signature

/// Identity of the network AS THE FACTORS SEE IT (D64 §4).
///
/// NAMED `FactorsSignature`, NOT `TopologySignature`, and the name is part of
/// the ruling: it deliberately EXCLUDES things that change the network while
/// leaving the factors identical — `shiftRad` above all. A general-purpose
/// "did the network change" key would invite caching base flows or
/// shift-derived quantities against it, and those DO depend on `shiftRad`.
///
/// Covers exactly the derived quantities the factors consume:
///   - `bSeries[k]` — which collapses `x` and `tap` into the one quantity that
///     matters, so `x=2,tap=1` and `x=1,tap=2` are correctly the same network;
///   - the `(from, to)` pair per branch, in branch-index order;
///   - the ternary classification isolated / reference-eligible / live.
///
/// Everything else falls out structurally: `r`, `b`, `g`, `ratingMva`,
/// `shiftRad`, `baseMVA`, bus loads and generator P are not inputs to any
/// factor value, and a `.pv` <-> `.pq` flip is not a distinction the build makes.
///
/// NOT Swift's `Hasher`: its per-process seed makes any derived value unstable
/// across launches. SHA-256 over a canonical encoding, doubles by BIT PATTERN,
/// `-0.0` normalised to `0.0`, non-finite rejected rather than hashed.
public struct FactorsSignature: Hashable, Sendable, CustomStringConvertible {
    public let digest: String
    public var description: String { "FactorsSignature(\(digest))" }

    /// Classification as the build sees it. `.pv` and `.pq` are the same value
    /// deliberately — the build does not distinguish them.
    enum BusClass: UInt8 { case isolated = 0, reference = 1, live = 2 }

    static func canonicalBytes(bSeries: [Double], froms: [Int], tos: [Int],
                               classes: [BusClass]) throws -> SHA256Digest {
        var hasher = SHA256()
        func absorb(_ v: Int) {
            withUnsafeBytes(of: Int64(v).littleEndian) { hasher.update(bufferPointer: $0) }
        }
        func absorb(_ v: UInt8) {
            withUnsafeBytes(of: v) { hasher.update(bufferPointer: $0) }
        }
        func absorb(_ v: Double) throws {
            guard v.isFinite else {
                throw SensitivityError.invalidNetworkParameter(
                    "a non-finite susceptance cannot be hashed")
            }
            let normalised = v == 0 ? 0.0 : v      // collapses -0.0 to 0.0
            withUnsafeBytes(of: normalised.bitPattern.littleEndian) {
                hasher.update(bufferPointer: $0)
            }
        }
        absorb(classes.count)
        for c in classes { absorb(c.rawValue) }
        absorb(bSeries.count)
        for k in 0..<bSeries.count {
            try absorb(bSeries[k])
            absorb(froms[k])
            absorb(tos[k])
        }
        return hasher.finalize()
    }

    /// Computed FROM THE NETWORK, every time. Never carried forward — that is
    /// the whole point of the guard.
    public static func of(_ net: BusBranchNetwork,
                          slack: SlackReference = .networkDefined) throws -> FactorsSignature {
        let (live, isReference) = try classify(net, slack: slack)
        let model = DCModel(net: net, live: live)
        let classes: [BusClass] = (0..<net.busCount).map {
            if !live[$0] { return .isolated }
            return isReference[$0] ? .reference : .live
        }
        let digest = try canonicalBytes(bSeries: model.bSeries,
                                        froms: net.branches.map(\.from),
                                        tos: net.branches.map(\.to),
                                        classes: classes)
        return FactorsSignature(
            digest: String(digest.compactMap { String(format: "%02x", $0) }
                .joined().prefix(32)))
    }
}

/// Live / reference classification, shared by the signature and the build so
/// the two can never disagree about what the network is.
func classify(_ net: BusBranchNetwork,
              slack: SlackReference) throws -> (live: [Bool], isReference: [Bool]) {
    let n = net.busCount
    var live = [Bool](repeating: false, count: n)
    var isReference = [Bool](repeating: false, count: n)
    for (i, bus) in net.buses.enumerated() {
        switch bus.type {
        case .slack: isReference[i] = true; live[i] = true
        case .pv, .pq: live[i] = true
        case .isolated: break
        }
    }

    switch slack {
    case .networkDefined:
        break
    case .distributed(let weights):
        // `.distributed` is a post-shift of the network-defined base, so its
        // BASE classification is the network's own — but the weights are
        // validated HERE, on construction of the classification, because an
        // invalid participation set must never reach a solve (D64 §2).
        try validate(weights, live: live, net: net)
    case .uniformlyDistributed(let buses):
        guard !buses.isEmpty else {
            throw SensitivityError.invalidReference("an empty absorption set leaves B singular")
        }
        var replacement = [Bool](repeating: false, count: n)
        for b in buses {
            guard b.index >= 0 && b.index < n else {
                throw SensitivityError.unknownBus(b)
            }
            guard live[b.index] else {
                throw SensitivityError.invalidReference(
                    "\(b) is isolated and cannot be a reference")
            }
            replacement[b.index] = true
        }
        isReference = replacement
    }

    guard isReference.contains(true) else {
        throw SensitivityError.invalidReference("the network has no reference bus")
    }
    return (live, isReference)
}

/// Participation-factor validation (D64 §2). Rejects, in order: unknown bus,
/// non-finite, negative, a set summing to zero, a bus that is isolated or whose
/// generation is out of service, and a set spanning more than one island.
///
/// **Island-spanning is rejected in v1 rather than handled.** Multi-reference
/// exists in part to serve several islands, and a participation set straddling
/// two of them is physically meaningless — there is no single compensating
/// injection. Per-island normalization is DEFERRED, not implemented (D64 §7).
func validate(_ weights: [BusID: Double], live: [Bool],
              net: BusBranchNetwork) throws {
    guard !weights.isEmpty else {
        throw SensitivityError.invalidParticipationFactors("the set is empty")
    }
    var total = 0.0
    for (bus, w) in weights {
        guard bus.index >= 0 && bus.index < net.busCount else {
            throw SensitivityError.unknownBus(bus)
        }
        guard w.isFinite else {
            throw SensitivityError.invalidParticipationFactors(
                "\(bus) carries a non-finite weight")
        }
        guard w >= 0 else {
            throw SensitivityError.invalidParticipationFactors(
                "\(bus) carries a negative weight (\(w))")
        }
        guard live[bus.index] else {
            throw SensitivityError.invalidParticipationFactors(
                "\(bus) is isolated and cannot participate")
        }
        let hasLiveGen = net.generators.contains { $0.inService && $0.bus == bus.index }
        guard hasLiveGen else {
            throw SensitivityError.invalidParticipationFactors(
                "\(bus) has no in-service generator and cannot participate")
        }
        total += w
    }
    guard total > 0 else {
        throw SensitivityError.invalidParticipationFactors("the weights sum to zero")
    }
    // Island-spanning check: every participating bus must reach every other
    // through in-service branches between live buses.
    let participants = weights.keys.map(\.index).sorted()
    if participants.count > 1 {
        let reachable = NetworkConnectivity.componentReachable(
            from: participants[0], in: net, live: live)
        for b in participants.dropFirst() where !reachable.contains(b) {
            throw SensitivityError.invalidParticipationFactors(
                "the participation set spans more than one island "
                + "(bus \(participants[0]) and bus \(b) are not connected); "
                + "per-island normalization is deferred, not implemented")
        }
    }
}
