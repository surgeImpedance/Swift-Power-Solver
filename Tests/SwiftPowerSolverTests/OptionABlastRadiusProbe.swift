import XCTest
@testable import SwiftPowerSolver

/// D80 (seventeenth sitting) — the blast-radius INSTRUMENT for the
/// negative-magnitude guard and Option A (the ZIP polynomial on |V|).
///
/// Opt-in (`SPS_BLAST=1`), asserts nothing about the numbers: it prints one
/// address-independent line per solve arm — an FNV-1a hash over the bit
/// patterns of every solution field plus the iteration count and the
/// convergence flag — so two runs at two revisions can be diffed line by
/// line. A converging case that a change touched shows up as a moved hash;
/// "no-op on every converging case" is then a diff with zero lines, not a
/// sentence. Arms: every reference network × {constant power, the §4.1 mix}
/// × {Q-limits off, on} × {NR, FDPF-BX, FDPF-XB, engine warm start}, plus the
/// two-bus reproducer ladder from the sixteenth sitting so the region the
/// change is FOR is printed beside the region it must not touch.
///
/// THE SENTINEL (eighteenth sitting, D80 item c). The XCTest harness prints
/// its test-status line concurrently with the test's own output and can
/// clobber the LAST line printed — measured in the seventeenth sitting: one
/// arm per run unreadable, a different one each run, which made "N arms
/// moved" need a clean-line filter. Each arm set therefore ends with a
/// sacrificial sentinel line, so what gets clobbered is the sentinel and
/// never an arm. The arm sets are built as line arrays, which is what lets
/// `ProbeSentinelTests` pin the sentinel without the env var.
final class OptionABlastRadiusProbe: XCTestCase {

    static let sentinel = "BLAST-END"

    private var enabled: Bool { ProcessInfo.processInfo.environment["SPS_BLAST"] == "1" }

    // MARK: - Hash

    private static func fnv(_ words: [UInt64]) -> String {
        var h: UInt64 = 0xcbf29ce484222325
        for w in words {
            for shift in stride(from: 0, to: 64, by: 8) {
                h ^= (w >> UInt64(shift)) & 0xff
                h = h &* 0x100000001b3
            }
        }
        return String(format: "%016llx", h)
    }

    static func line(_ tag: String, _ s: PowerFlowSolution) -> String {
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

    static func mixed(_ net: BusBranchNetwork) -> BusBranchNetwork {
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

    // MARK: - Arm sets (line arrays; the env-gated tests print them)

    private static func arms(_ name: String, _ plain: BusBranchNetwork) -> [String] {
        var out: [String] = []
        for (model, net) in [("cp", plain), ("mix", mixed(plain))] {
            for q in [false, true] {
                let tag = "\(name)/\(model)/q\(q ? "on" : "off")"
                out.append(line("\(tag)/nr", NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(enforceQLimits: q))))
                if !q {
                    for v in [FDPFVariant.bx, .xb] {
                        var o = PowerFlowOptions(maxIterations: 200); o.fdpfVariant = v
                        out.append(line("\(tag)/fdpf-\(v.rawValue)", FastDecoupledSolver().solve(net, options: o)))
                    }
                }
                var e = PowerFlowOptions(enforceQLimits: q); e.method = .fastDecoupledWarmStart; e.autoFallback = true
                out.append(line("\(tag)/engine-warm", PowerFlowEngine().solve(net, options: e)))
            }
        }
        return out
    }

    /// Every reference network's arms, ending with the sentinel.
    static func referenceArmLines() throws -> [String] {
        var out: [String] = []
        for name in ["case14", "case39", "case118"] {
            out += arms(name, try ReferenceCase.load(name).network())
        }
        // `FDPFReferenceCase.load` prepends "fdpf_" itself. Passing the full
        // resource name here threw an XCTSkip MID-TEST on every seventeenth-
        // sitting run (after the three cases above had printed), so the two
        // FDPF cases were never in the "36 reference arms" — see D80,
        // eighteenth sitting. A self-skipping arm set is not a measurement.
        for name in ["case30", "case300"] {
            out += arms("fdpf_\(name)", try FDPFReferenceCase.load(name).network())
        }
        out.append(sentinel)
        return out
    }

    /// The two-bus ladder's arms, ending with the sentinel.
    static func twoBusLadderLines() -> [String] {
        var out: [String] = []
        let models: [(String, Double, Double, Double, Double)] = [
            ("P", 0, 0, 0, 0), ("mix", 0.3, 0.3, 0.5, 0.3), ("Z", 1, 0, 1, 0),
            ("I", 0, 1, 0, 1), ("halfI", 0, 0.5, 0, 0.5), ("ZPnoI", 0.5, 0, 0.5, 0)]
        for (label, zP, iP, zQ, iQ) in models {
            for mw in [300.0, 600, 900, 1000, 1100, 1200, 1300, 1500, 2000, 4000] {
                let net = twoBus(loadMw: mw, zP: zP, iP: iP, zQ: zQ, iQ: iQ)
                let tag = "twobus/\(label)/\(Int(mw))"
                for q in [false, true] {
                    out.append(line("\(tag)/q\(q ? "on" : "off")/nr", NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(enforceQLimits: q))))
                }
                var o = PowerFlowOptions(maxIterations: 200)
                out.append(line("\(tag)/fdpf-bx", FastDecoupledSolver().solve(net, options: o)))
                o.fdpfVariant = .xb
                out.append(line("\(tag)/fdpf-xb", FastDecoupledSolver().solve(net, options: o)))
                var e = PowerFlowOptions(); e.method = .fastDecoupledWarmStart; e.autoFallback = true
                out.append(line("\(tag)/engine-warm", PowerFlowEngine().solve(net, options: e)))
                var a = PowerFlowOptions(); a.autoFallback = true
                out.append(line("\(tag)/engine-nr-auto", PowerFlowEngine().solve(net, options: a)))
            }
        }
        out.append(sentinel)
        return out
    }

    // MARK: - The env-gated runs

    /// MEASURED (eighteenth sitting): with the sentinel in place one arm line
    /// was STILL cut — "BLAST twobus/ZPn" + the harness's status line — and
    /// it was not the last line of the set. The mechanism is stdout's block
    /// buffer flushing at a boundary in the middle of a line while the
    /// harness writes its own line to the same pipe; a trailing sentinel
    /// cannot help with that. Line-buffering stdout makes every arm line a
    /// single write, and the run then reads 420 clean lines of 420. The
    /// sentinel stays as the end-of-set marker a diff can check for.
    private func lineBufferStdout() { setvbuf(stdout, nil, _IOLBF, 0) }

    func testReferenceArms() throws {
        try XCTSkipUnless(enabled, "opt-in: SPS_BLAST=1")
        lineBufferStdout()
        for l in try Self.referenceArmLines() { print(l) }
    }

    func testTwoBusLadder() throws {
        try XCTSkipUnless(enabled, "opt-in: SPS_BLAST=1")
        lineBufferStdout()
        for l in Self.twoBusLadderLines() { print(l) }
    }
}

/// D80 item (c), eighteenth sitting: each arm set ends with exactly one
/// sentinel line and nothing else is one. Ungated, so the pin runs in every
/// suite; written before the sentinel was appended and measured to fail.
final class ProbeSentinelTests: XCTestCase {

    private func check(_ lines: [String], _ set: String) {
        XCTAssertGreaterThan(lines.count, 1, "\(set): the arm set is not empty")
        XCTAssertEqual(lines.last, OptionABlastRadiusProbe.sentinel, "\(set): the last line is the sentinel")
        XCTAssertEqual(lines.filter { $0 == OptionABlastRadiusProbe.sentinel }.count, 1, "\(set): exactly one sentinel")
        XCTAssertTrue(lines.dropLast().allSatisfy { $0.hasPrefix("BLAST ") }, "\(set): every other line is an arm")
    }

    func testEveryArmSetEndsWithTheSentinel() throws {
        check(try OptionABlastRadiusProbe.referenceArmLines(), "reference")
        check(OptionABlastRadiusProbe.twoBusLadderLines(), "two-bus ladder")
    }
}
