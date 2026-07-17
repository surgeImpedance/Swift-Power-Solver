import XCTest
@testable import SwiftPowerSolver

// Piece 0 oracle: YbusBuilder output must match pandapower's own Ybus
// (net._ppc.internal.Ybus) entry-for-entry on the IEEE cases.
final class YbusTests: XCTestCase {

    /// Absolute tolerance per entry, scaled by the entry magnitude. pandapower
    /// runs the same float64 formulas, so agreement should be ~1e-13; 1e-9
    /// leaves room for ordering differences in the sums.
    private func tolerance(for magnitude: Double) -> Double {
        1e-9 * max(1.0, magnitude)
    }

    private func compare(case name: String) throws {
        let ref = try ReferenceCase.load(name)
        let ybus = YbusBuilder.build(ref.network())

        XCTAssertEqual(ybus.n, ref.ybus.n, "\(name): bus count")

        // Union of coordinates from both matrices; missing entries are zero.
        var refEntries: [Int: ComplexD] = [:]
        for k in 0..<ref.ybus.row.count {
            refEntries[ref.ybus.row[k] * ybus.n + ref.ybus.col[k]] =
                ComplexD(ref.ybus.g[k], ref.ybus.b[k])
        }
        var ourEntries: [Int: ComplexD] = [:]
        for e in ybus.coordinateEntries() {
            ourEntries[e.row * ybus.n + e.col] = e.value
        }

        var maxErr = 0.0, sumErr = 0.0
        var worst = (i: -1, j: -1, ours: ComplexD.zero, ref: ComplexD.zero)
        let keys = Set(refEntries.keys).union(ourEntries.keys)
        for key in keys {
            let r = refEntries[key] ?? .zero
            let o = ourEntries[key] ?? .zero
            let err = (o - r).magnitude
            sumErr += err
            if err > maxErr {
                maxErr = err
                worst = (key / ybus.n, key % ybus.n, o, r)
            }
            XCTAssertLessThanOrEqual(
                err, tolerance(for: r.magnitude),
                """
                \(name): Ybus[\(key / ybus.n),\(key % ybus.n)] mismatch \
                ours=(\(o.re), \(o.im)) pandapower=(\(r.re), \(r.im))
                """)
        }
        print(String(format: "%@ Ybus vs pandapower: %d entries, max |ΔY| %.3e pu (at [%d,%d]), mean %.3e pu",
                     name, keys.count, maxErr, worst.i, worst.j,
                     keys.isEmpty ? 0 : sumErr / Double(keys.count)))
    }

    func testCase14() throws { try compare(case: "case14") }
    func testCase39() throws { try compare(case: "case39") }
    func testCase118() throws { try compare(case: "case118") }

    // A hand-checkable 2-bus sanity case: one line, no tap, with charging and
    // a bus shunt — verifies the pi-model wiring independent of pandapower.
    func testTwoBusPiModel() {
        let net = BusBranchNetwork(
            baseMVA: 100,
            buses: [
                .init(type: .slack, baseKv: 110),
                .init(type: .pq, baseKv: 110, bsPu: 0.25),
            ],
            branches: [.init(from: 0, to: 1, r: 0.01, x: 0.1, b: 0.04)],
            generators: [.init(bus: 0, pPu: 0, vSetPu: 1.0)])
        let y = YbusBuilder.build(net)

        let ys = ComplexD(1, 0) / ComplexD(0.01, 0.1)   // series admittance
        XCTAssertEqual(y[0, 0].re, ys.re, accuracy: 1e-14)
        XCTAssertEqual(y[0, 0].im, ys.im + 0.02, accuracy: 1e-14)
        XCTAssertEqual(y[0, 1].re, -ys.re, accuracy: 1e-14)
        XCTAssertEqual(y[0, 1].im, -ys.im, accuracy: 1e-14)
        XCTAssertEqual(y[1, 0].re, -ys.re, accuracy: 1e-14)
        XCTAssertEqual(y[1, 1].im, ys.im + 0.02 + 0.25, accuracy: 1e-14)
    }

    // Tap + phase shift: yft/ytf must be asymmetric under a shift.
    func testTapAndShiftAsymmetry() {
        let branch = BusBranchNetwork.Branch(
            from: 0, to: 1, r: 0, x: 0.2, b: 0, tap: 0.978, shiftRad: 0.1)
        let y = YbusBuilder.admittance(of: branch)
        let ys = ComplexD(1, 0) / ComplexD(0, 0.2)
        // |yft| == |ytf| == |ys|/tap, but rotated opposite ways.
        XCTAssertEqual(y.yft.magnitude, ys.magnitude / 0.978, accuracy: 1e-12)
        XCTAssertEqual(y.ytf.magnitude, ys.magnitude / 0.978, accuracy: 1e-12)
        XCTAssertNotEqual(y.yft.re, y.ytf.re)
        // yff scales with 1/tap², ytt does not.
        XCTAssertEqual(y.yff.im, ys.im / (0.978 * 0.978), accuracy: 1e-12)
        XCTAssertEqual(y.ytt.im, ys.im, accuracy: 1e-12)
    }
}
