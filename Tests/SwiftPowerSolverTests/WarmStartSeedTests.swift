import XCTest
@testable import SwiftPowerSolver

/// D80 item (d), eighteenth sitting — a PIN, not a witness: Newton's
/// warm-start guard takes only finite, POSITIVE seed magnitudes and falls
/// back to flat per bus for the rest (`PowerFlowOptions.initialVmPu`'s
/// contract). Nothing covered it directly; the seventeenth sitting noted it
/// as "worth knowing when reading a warm-start trajectory" (a negative or
/// zero entry in a seed is silently a flat start for that bus, so a seeded
/// arm can be a flat arm without saying so). Pinned bit for bit against the
/// flat solve; the positive-seed control shows the seed is otherwise live.
final class WarmStartSeedTests: XCTestCase {

    func testNonPositiveAndNonFiniteSeedEntriesFallBackToFlatPerBus() throws {
        let net = try ReferenceCase.load("case14").network()
        let flat = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions())
        XCTAssertTrue(flat.converged)
        for bad in [-1.0, -0.3, 0.0, .nan, .infinity] {
            var o = PowerFlowOptions()
            o.initialVmPu = [Double](repeating: bad, count: net.busCount)
            let seeded = NewtonRaphsonSolver().solve(net, options: o)
            XCTAssertEqual(seeded.iterations, flat.iterations, "seed \(bad): same trajectory as flat")
            XCTAssertEqual(seeded.vmPu.map(\.bitPattern), flat.vmPu.map(\.bitPattern), "seed \(bad): bit-identical to flat")
            XCTAssertEqual(seeded.vaRad.map(\.bitPattern), flat.vaRad.map(\.bitPattern), "seed \(bad)")
        }
    }

    /// Control: a positive seed IS taken — seeding from the solution
    /// converges in fewer iterations than flat, so the pin above is not
    /// vacuous.
    func testAPositiveSeedIsLive() throws {
        let net = try ReferenceCase.load("case14").network()
        let flat = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions())
        var o = PowerFlowOptions()
        o.initialVmPu = flat.vmPu; o.initialVaRad = flat.vaRad
        let seeded = NewtonRaphsonSolver().solve(net, options: o)
        XCTAssertTrue(seeded.converged)
        XCTAssertLessThan(seeded.iterations, flat.iterations, "seeded \(seeded.iterations) vs flat \(flat.iterations)")
    }
}
