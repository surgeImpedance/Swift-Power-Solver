import XCTest
@testable import SwiftPowerSolver

/// D80 (seventeenth sitting) — the blast-radius INSTRUMENT for the
/// negative-magnitude guard and Option A (the ZIP polynomial on |V|).
///
/// Opt-in (`SPS_BLAST=1`), asserts nothing about the numbers: it prints one
/// address-independent line per solve arm — an FNV-1a hash over the bit
/// patterns of every solution field plus the iteration count and the
/// convergence flag — so two runs at two revisions can be diffed line by
/// line. A converging case that Option A touched shows up as a moved hash;
/// "no-op on every converging case" is then a diff with zero lines, not a
/// sentence. Arms: every reference network × {constant power, the §4.1 mix}
/// × {Q-limits off, on} × {NR, FDPF-BX, FDPF-XB, engine warm start}, plus the
/// two-bus reproducer ladder from the sixteenth sitting so the region the
/// change is FOR is printed beside the region it must not touch.
final class OptionABlastRadiusProbe: XCTestCase {

    private var enabled: Bool { ProcessInfo.processInfo.environment["SPS_BLAST"] == "1" }

    // MARK: - Hash

    private func fnv(_ words: [UInt64]) -> String {
        var h: UInt64 = 0xcbf29ce484222325
        for w in words {
            for shift in stride(from: 0, to: 64, by: 8) {
                h ^= (w >> UInt64(shift)) & 0xff
                h = h &* 0x100000001b3
            }
        }
        return String(format: "%016llx", h)
    }

    private func line(_ tag: String, _ s: PowerFlowSolution) -> String {
        var words: [UInt64] = [s.converged ? 1 : 0, UInt64(s.iterations)]
        words += s.vmPu.map(\.bitPattern)
        words += s.vaRad.map(\.bitPattern)
        words += s.genPPu.map(\.bitPattern)
        words += s.genQPu.map(\.bitPattern)
        words += s.loadPPu.map(\.bitPattern)
        words += s.loadQPu.map(\.bitPattern)
        for f in s.branchFlows { words += [f.pFromPu.bitPattern, f.qFromPu.bitPattern, f.pToPu.bitPattern, f.qToPu.bitPattern] }
        words += s.pinnedGenIndices.sorted().map { UInt64($0) }
        let vmin = s.vmPu.filter(\.isFinite).min() ?? .nan
        return "BLAST \(tag) hash=\(fnv(words)) conv=\(s.converged) it=\(s.iterations) "
            + "vmin=\(String(format: "%.6f", vmin)) pinned=\(s.pinnedGenIndices.count)"
            + (s.failureReason.map { " reason=\"\($0.prefix(90))\"" } ?? "")
    }

    // MARK: - Networks

    private func mixed(_ net: BusBranchNetwork) -> BusBranchNetwork {
        var z = net
        for i in z.buses.indices where z.buses[i].pLoadPu != 0 || z.buses[i].qLoadPu != 0 {
            z.buses[i].pLoadZPu = 0.3 * z.buses[i].pLoadPu
            z.buses[i].pLoadIPu = 0.3 * z.buses[i].pLoadPu
            z.buses[i].qLoadZPu = 0.5 * z.buses[i].qLoadPu
            z.buses[i].qLoadIPu = 0.3 * z.buses[i].qLoadPu
        }
        return z
    }

    /// The sixteenth sitting's two-bus reproducer in package units: slack at
    /// 1.0 pu, one load bus across a 30 km, 0.05 + j0.3 Ω/km, 115 kV line.
    static func twoBus(loadMw: Double, zP: Double, iP: Double, zQ: Double, iQ: Double) -> BusBranchNetwork {
        let base = 100.0, zb = 115.0 * 115.0 / base
        let p = loadMw / base, q = 0.3 * loadMw / base
        return BusBranchNetwork(
            baseMVA: base,
            buses: [.init(type: .slack, baseKv: 115),
                    .init(type: .pq, baseKv: 115, pLoadPu: p, qLoadPu: q,
                          pLoadZPu: zP * p, pLoadIPu: iP * p, qLoadZPu: zQ * q, qLoadIPu: iQ * q)],
            branches: [.init(from: 0, to: 1, r: 30 * 0.05 / zb, x: 30 * 0.3 / zb)],
            generators: [.init(bus: 0, pPu: 0, vSetPu: 1.0)])
    }

    private func arms(_ name: String, _ plain: BusBranchNetwork) {
        for (model, net) in [("cp", plain), ("mix", mixed(plain))] {
            for q in [false, true] {
                let tag = "\(name)/\(model)/q\(q ? "on" : "off")"
                print(line("\(tag)/nr", NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(enforceQLimits: q))))
                if !q {
                    for v in [FDPFVariant.bx, .xb] {
                        var o = PowerFlowOptions(maxIterations: 200); o.fdpfVariant = v
                        print(line("\(tag)/fdpf-\(v.rawValue)", FastDecoupledSolver().solve(net, options: o)))
                    }
                }
                var e = PowerFlowOptions(enforceQLimits: q); e.method = .fastDecoupledWarmStart; e.autoFallback = true
                print(line("\(tag)/engine-warm", PowerFlowEngine().solve(net, options: e)))
            }
        }
    }

    func testReferenceArms() throws {
        try XCTSkipUnless(enabled, "opt-in: SPS_BLAST=1")
        for name in ["case14", "case39", "case118"] {
            arms(name, try ReferenceCase.load(name).network())
        }
        for name in ["fdpf_case30", "fdpf_case300"] {
            arms(name, try FDPFReferenceCase.load(name).network())
        }
    }

    func testTwoBusLadder() throws {
        try XCTSkipUnless(enabled, "opt-in: SPS_BLAST=1")
        let models: [(String, Double, Double, Double, Double)] = [
            ("P", 0, 0, 0, 0), ("mix", 0.3, 0.3, 0.5, 0.3), ("Z", 1, 0, 1, 0),
            ("I", 0, 1, 0, 1), ("halfI", 0, 0.5, 0, 0.5), ("ZPnoI", 0.5, 0, 0.5, 0)]
        for (label, zP, iP, zQ, iQ) in models {
            for mw in [300.0, 600, 900, 1000, 1100, 1200, 1300, 1500, 2000, 4000] {
                let net = Self.twoBus(loadMw: mw, zP: zP, iP: iP, zQ: zQ, iQ: iQ)
                let tag = "twobus/\(label)/\(Int(mw))"
                for q in [false, true] {
                    print(line("\(tag)/q\(q ? "on" : "off")/nr", NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(enforceQLimits: q))))
                }
                var o = PowerFlowOptions(maxIterations: 200)
                print(line("\(tag)/fdpf-bx", FastDecoupledSolver().solve(net, options: o)))
                o.fdpfVariant = .xb
                print(line("\(tag)/fdpf-xb", FastDecoupledSolver().solve(net, options: o)))
                var e = PowerFlowOptions(); e.method = .fastDecoupledWarmStart; e.autoFallback = true
                print(line("\(tag)/engine-warm", PowerFlowEngine().solve(net, options: e)))
                var a = PowerFlowOptions(); a.autoFallback = true
                print(line("\(tag)/engine-nr-auto", PowerFlowEngine().solve(net, options: a)))
            }
        }
    }
}
