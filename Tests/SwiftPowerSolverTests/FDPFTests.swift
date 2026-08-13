import XCTest
@testable import SwiftPowerSolver

// Gate 1 — FDPF correctness. Both variants, flat start, tol 1e-8, on every
// bundled reference case: the converged FDPF solution must land on the NR
// solution to within 1e-6 pu / 1e-6 rad. (The pegase-scale cases run app-side
// in substation-lab, which owns those fixtures.)
final class FDPFTests: XCTestCase {

    /// Cases with a slim `fdpf_<name>.json` params+solution fixture are loaded
    /// through `FDPFReferenceCase`; the full-dump cases through `ReferenceCase`.
    private let fullDumpCases = ["case14", "case39", "case118"]
    private let slimDumpCases = ["case30", "case300"]

    private func networks() throws -> [(name: String, net: BusBranchNetwork)] {
        var nets: [(String, BusBranchNetwork)] = []
        for name in fullDumpCases {
            nets.append((name, try ReferenceCase.load(name).network()))
        }
        for name in slimDumpCases {
            nets.append((name, try FDPFReferenceCase.load(name).network()))
        }
        return nets
    }

    func testFDPFMatchesNewtonRaphsonBothVariants() throws {
        let options = PowerFlowOptions(tolerancePu: 1e-8)
        for (name, net) in try networks() {
            let nr = NewtonRaphsonSolver().solve(net, options: options)
            XCTAssertTrue(nr.converged, "\(name): NR did not converge")
            for variant in FDPFVariant.allCases {
                var opts = options
                opts.fdpfVariant = variant
                let fd = FastDecoupledSolver().solve(net, options: opts)
                XCTAssertTrue(fd.converged,
                              "\(name)/\(variant): FDPF did not converge from flat start "
                              + "(\(fd.failureReason ?? "-"))")
                guard fd.converged else { continue }
                XCTAssertEqual(fd.solutionPath, .fdpf)

                var dV = 0.0, dA = 0.0
                for i in 0..<net.busCount where nr.vmPu[i].isFinite {
                    dV = max(dV, abs(fd.vmPu[i] - nr.vmPu[i]))
                    dA = max(dA, abs(fd.vaRad[i] - nr.vaRad[i]))
                }
                XCTAssertLessThan(dV, 1e-6, "\(name)/\(variant): max|ΔV|")
                XCTAssertLessThan(dA, 1e-6, "\(name)/\(variant): max|Δθ|")
                print(String(format: "FDPF %@ %@: %d rounds, max|ΔV|=%.2e pu, max|Δθ|=%.2e rad (NR: %d it)",
                             name, variant.rawValue.uppercased(), fd.iterations, dV, dA, nr.iterations))
            }
        }
    }

    /// The pandapower-solved slim fixtures double-check the fixture conversion
    /// itself: NR on the slim network must match pandapower, same bar the full
    /// dumps are held to.
    func testSlimFixturesAgreeWithPandapower() throws {
        for name in slimDumpCases {
            let ref = try FDPFReferenceCase.load(name)
            let net = ref.network()
            let sol = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(tolerancePu: 1e-10))
            XCTAssertTrue(sol.converged, "\(name): NR did not converge")
            let vm = ErrorStats(reference: ref.solution.vmPu, computed: sol.vmPu)
            let va = ErrorStats(reference: ref.solution.vaDeg.map { $0 * .pi / 180 },
                                computed: sol.vaRad)
            XCTAssertLessThan(vm.maxError, 1e-8, "\(name): vm vs pandapower")
            XCTAssertLessThan(va.maxError, 1e-8, "\(name): va vs pandapower")
        }
    }
}
