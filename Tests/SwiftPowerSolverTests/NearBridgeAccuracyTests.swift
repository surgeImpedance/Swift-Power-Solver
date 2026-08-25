import Foundation
import XCTest
@testable import SwiftPowerSolver

/// E2: how many digits of the LODF column actually survive at a near-bridge?
///
/// The derived figure — "about 4 of 16 digits lost at |1-h| = 5.3e-4" — is the
/// REPRESENTATION-ONLY floor. It assumes `h` is exact to machine epsilon before
/// the subtraction, and it is not: `h` carries the solve's accumulated error,
/// bounded at case9241 by a worst residual of 9.734e-13. Cancellation costs
/// digits relative to `h`'s ACTUAL accuracy, so the true loss is larger.
///
/// Rather than estimate it, this measures it, with gate 6.5's own instrument
/// restricted to the worst near-bridge branches: remove the branch, re-solve DC,
/// and compare the LODF superposition against the re-solve. The DC solver is an
/// independent path from the factors (it does not consult them), and it is
/// itself pinned against pandapower by DCPowerFlowTests — so the deviation here
/// is the empirical answer.
///
/// Typical branches are measured alongside as the control. Without them a large
/// near-bridge error is unattributable: it could be the superposition being
/// wrong everywhere.
final class NearBridgeAccuracyTests: XCTestCase {

    private func note(_ m: String) {
        FileHandle.standardError.write(Data((m + "\n").utf8))
    }

    private struct Probe {
        var branch: Int
        var deviationFromH: Double     // |1 - h_k|
        var maxAbsErrorPu: Double
        var relativeError: Double
        var survivingDigits: Double
    }

    /// One outage: LODF superposition vs. a real DC re-solve with the branch
    /// out. Returns nil when the re-solve does not converge.
    private func probe(_ net: BusBranchNetwork, _ factors: DistributionFactors,
                       base: PowerFlowSolution, k: Int) -> Probe? {
        var outaged = net
        outaged.branches[k].inService = false
        let post = DCPowerFlowSolver().solve(outaged)
        guard post.converged else { return nil }

        let basePu = base.branchFlows.map(\.pFromPu)
        let actualPu = post.branchFlows.map(\.pFromPu)
        let fk = basePu[k]

        var maxAbs = 0.0
        var scale = 0.0
        for m in 0..<net.branches.count where m != k {
            let predicted = basePu[m] + factors.lodf(monitored: m, outaged: k) * fk
            maxAbs = max(maxAbs, abs(predicted - actualPu[m]))
            scale = max(scale, abs(actualPu[m]))
        }
        let br = net.branches[k]
        let h = factors.ptdf(branch: k, bus: br.from) - factors.ptdf(branch: k, bus: br.to)
        let rel = scale > 0 ? maxAbs / scale : .nan
        return Probe(branch: k, deviationFromH: abs(1 - h),
                     maxAbsErrorPu: maxAbs, relativeError: rel,
                     survivingDigits: rel > 0 ? -log10(rel) : .infinity)
    }

    private func measure(_ name: String, _ net: BusBranchNetwork) {
        let factors = DistributionFactors.build(net)
        let base = DCPowerFlowSolver().solve(net)
        guard base.converged else { return XCTFail("\(name): base DC solve failed") }
        let bridges = ConnectivityIslandingTests.bridges(net)

        // Non-bridges ranked by how close h comes to 1 — the near-bridges.
        var candidates: [(k: Int, dev: Double)] = []
        for k in 0..<net.branches.count
        where net.branches[k].inService && !bridges[k] && factors.ptdf(branch: k, bus: 0) == factors.ptdf(branch: k, bus: 0) {
            let br = net.branches[k]
            let h = factors.ptdf(branch: k, bus: br.from) - factors.ptdf(branch: k, bus: br.to)
            let dev = abs(1 - h)
            guard dev.isFinite, dev > 0 else { continue }
            candidates.append((k, dev))
        }
        candidates.sort { $0.dev < $1.dev }
        guard !candidates.isEmpty else { return }

        let worst = candidates.prefix(5)
        let mid = candidates.count / 2
        let typical = candidates[max(0, mid - 1)...min(candidates.count - 1, mid + 1)]

        note("E2 \(name): NEAR-BRIDGES (smallest |1-h| among non-bridges)")
        for c in worst {
            guard let p = probe(net, factors, base: base, k: c.k) else {
                note("E2 \(name):   branch \(c.k) re-solve did not converge")
                continue
            }
            note(String(format: "E2 %@:   branch %-6d |1-h|=%.4e  maxAbsErr=%.4e pu  "
                        + "rel=%.4e  surviving digits ~ %.1f",
                        name, p.branch, p.deviationFromH, p.maxAbsErrorPu,
                        p.relativeError, p.survivingDigits))
        }
        note("E2 \(name): TYPICAL BRANCHES (median |1-h|) — the control")
        for c in typical {
            guard let p = probe(net, factors, base: base, k: c.k) else { continue }
            note(String(format: "E2 %@:   branch %-6d |1-h|=%.4e  maxAbsErr=%.4e pu  "
                        + "rel=%.4e  surviving digits ~ %.1f",
                        name, p.branch, p.deviationFromH, p.maxAbsErrorPu,
                        p.relativeError, p.survivingDigits))
        }
    }

    func testReferenceCase() throws {
        measure("case118", try ReferenceCase.load("case118").network())
    }

    func testAtScale() throws {
        guard let paths = ProcessInfo.processInfo.environment["SPS_FACTORS_CASES"] else {
            XCTFail("SPS_FACTORS_CASES unset — E2 cannot measure at scale.")
            return
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        for path in paths.split(separator: ",").map(String.init) {
            let f = try decoder.decode(ScaleNet.self,
                                       from: Data(contentsOf: URL(fileURLWithPath: path)))
            measure(f.name, f.network())
        }
    }

    private struct ScaleNet: Decodable {
        struct B: Decodable { var i: Int; var type: Int; var pdMw, qdMvar, gsMw, bsMvar, baseKv: Double }
        struct R: Decodable { var f, t: Int; var r, x, b, g, tap, shiftDeg: Double; var status: Int }
        struct G: Decodable { var bus: Int; var pgMw, qmaxMvar, qminMvar, vgPu, vaDeg: Double; var status: Int }
        var name: String
        var baseMva: Double
        var buses: [B]
        var branches: [R]
        var gens: [G]

        func network() -> BusBranchNetwork {
            BusBranchNetwork(
                baseMVA: baseMva,
                buses: buses.map {
                    BusBranchNetwork.Bus(type: BusBranchNetwork.BusType(rawValue: $0.type) ?? .pq,
                                         baseKv: $0.baseKv,
                                         pLoadPu: $0.pdMw / baseMva, qLoadPu: $0.qdMvar / baseMva,
                                         gsPu: $0.gsMw / baseMva, bsPu: $0.bsMvar / baseMva)
                },
                branches: branches.map {
                    BusBranchNetwork.Branch(from: $0.f, to: $0.t, r: $0.r, x: $0.x, b: $0.b, g: $0.g,
                                            tap: $0.tap <= 0 ? 1.0 : $0.tap,
                                            shiftRad: $0.shiftDeg * .pi / 180,
                                            inService: $0.status == 1)
                },
                generators: gens.map {
                    BusBranchNetwork.Generator(bus: $0.bus, pPu: $0.pgMw / baseMva, vSetPu: $0.vgPu,
                                               vaRefRad: $0.vaDeg * .pi / 180,
                                               qMinPu: $0.qminMvar / baseMva,
                                               qMaxPu: $0.qmaxMvar / baseMva,
                                               inService: $0.status == 1)
                })
        }
    }
}
