import XCTest
@testable import SwiftPowerSolver

// Piece 4 oracle: IEC 60909 short-circuit currents must match pandapower's
// calc_sc on the purpose-built reference networks. Both sides solve identical
// per-unit data, so — like Pieces 0–3 — agreement is expected near machine
// precision, not merely under tolerance.
final class ShortCircuitTests: XCTestCase {

    private let currentTol = 1e-6     // kA
    private let ohmTol = 1e-6         // ohm
    private let mvaTol = 1e-4         // MVA

    private func validate(fault: ShortCircuitOptions.FaultType,
                          key: String) throws {
        let ref = try ShortCircuitReference.load()
        var worstIkss = 0.0, worstIp = 0.0, worstIth = 0.0
        var worstNet = ""

        for net in ref.networks {
            guard let refFaults = net.results[key] else {
                XCTFail("\(net.name): no \(key) results"); continue
            }
            let bbn = net.network()
            var options = ShortCircuitOptions()
            options.voltageFactorC = net.c
            options.faultType = fault
            options.systemFrequencyHz = net.freqHz
            options.faultDurationS = net.tkS
            let result = ShortCircuitAnalyzer().faults(bbn, options: options)

            XCTAssertEqual(result.buses.count, refFaults.count,
                           "\(net.name): bus count")

            var ikssErr = 0.0, ipErr = 0.0, ithErr = 0.0
            for (i, refFault) in refFaults.enumerated() {
                let got = result.buses[i]
                guard let rf = refFault else {
                    // pandapower NaN here — must be a reported non-solve, not a number.
                    XCTAssertNotEqual(got.outcome, .solved,
                                      "\(net.name) bus \(i): expected edge outcome, got a value")
                    XCTAssertTrue(got.ikssKa.isNaN,
                                  "\(net.name) bus \(i): non-solved bus must have NaN current")
                    continue
                }
                XCTAssertEqual(got.outcome, .solved, "\(net.name) bus \(i)")

                XCTAssertEqual(got.ikssKa, rf.ikssKa, accuracy: currentTol,
                               "\(net.name) bus \(i): Ikss''")
                ikssErr = max(ikssErr, abs(got.ikssKa - rf.ikssKa))
                XCTAssertEqual(got.rkOhm, rf.rkOhm, accuracy: ohmTol,
                               "\(net.name) bus \(i): Rk")
                XCTAssertEqual(got.xkOhm, rf.xkOhm, accuracy: ohmTol,
                               "\(net.name) bus \(i): Xk")
                if let ip = rf.ipKa {
                    XCTAssertEqual(got.ipKa, ip, accuracy: currentTol,
                                   "\(net.name) bus \(i): ip")
                    ipErr = max(ipErr, abs(got.ipKa - ip))
                }
                if let ith = rf.ithKa {
                    XCTAssertEqual(got.ithKa, ith, accuracy: currentTol,
                                   "\(net.name) bus \(i): Ith")
                    ithErr = max(ithErr, abs(got.ithKa - ith))
                }
                if let skss = rf.skssMva {
                    XCTAssertEqual(got.skssMva, skss, accuracy: mvaTol,
                                   "\(net.name) bus \(i): Skss")
                }
            }
            print(String(format: "  %@ %@: max|ΔIkss| %.3e kA, max|Δip| %.3e kA, "
                         + "max|ΔIth| %.3e kA", net.name, key, ikssErr, ipErr, ithErr))
            if ikssErr > worstIkss { worstIkss = ikssErr; worstNet = net.name }
            worstIp = max(worstIp, ipErr); worstIth = max(worstIth, ithErr)
        }

        print(String(format: "%@ short-circuit vs pandapower calc_sc: worst |ΔIkss| "
                     + "%.3e kA (%@), |Δip| %.3e kA, |ΔIth| %.3e kA",
                     key, worstIkss, worstNet, worstIp, worstIth))
    }

    func testThreePhase() throws { try validate(fault: .threePhase, key: "3ph") }
    func testTwoPhase() throws { try validate(fault: .twoPhase, key: "2ph") }

    // MARK: - Edge cases as reported outcomes

    func testEdgeCaseOutcomes() throws {
        let ref = try ShortCircuitReference.load()
        guard let edge = ref.networks.first(where: { $0.name == "edge_cases" }) else {
            throw XCTSkip("no edge_cases network")
        }
        let result = ShortCircuitAnalyzer().faults(edge.network())
        // bus 2 is isolated; buses 3,4 form a source-less component.
        XCTAssertEqual(result.buses[2].outcome, .isolatedBus)
        XCTAssertEqual(result.buses[3].outcome, .noSourceFeeding)
        XCTAssertEqual(result.buses[4].outcome, .noSourceFeeding)
        XCTAssertEqual(result.buses[0].outcome, .solved)
        for b in result.buses where b.outcome != .solved {
            XCTAssertTrue(b.ikssKa.isNaN && b.ipKa.isNaN)
        }
    }

    /// A network with no fault source at all: every bus reports noSourceFeeding,
    /// nothing crashes.
    func testNoSourceAnywhere() {
        let net = BusBranchNetwork(
            baseMVA: 100,
            buses: [.init(type: .pq, baseKv: 110), .init(type: .pq, baseKv: 110)],
            branches: [.init(from: 0, to: 1, r: 0.01, x: 0.05)],
            generators: [])   // no sources
        let result = ShortCircuitAnalyzer().faults(net)
        XCTAssertTrue(result.buses.allSatisfy { $0.outcome == .noSourceFeeding })
    }

    /// The 2-phase current is √3/2 of the three-phase value at every fed bus.
    func testTwoPhaseIsScaledThreePhase() throws {
        let ref = try ShortCircuitReference.load()
        let net = ref.networks.first { $0.name == "meshed_two_source" }!.network()
        var opts = ShortCircuitOptions()
        let r3 = ShortCircuitAnalyzer().faults(net, options: opts)
        opts.faultType = .twoPhase
        let r2 = ShortCircuitAnalyzer().faults(net, options: opts)
        for i in 0..<net.busCount where r3.buses[i].outcome == .solved {
            XCTAssertEqual(r2.buses[i].ikssKa, sqrt(3) / 2 * r3.buses[i].ikssKa,
                           accuracy: 1e-12, "bus \(i)")
        }
    }
}
