import XCTest
@testable import SwiftPowerSolver

// Distributed-slack oracle. Gate 1 (single-slack bit-identical) is proven by
// the unchanged Piece 0-4 numbers; this file covers the distributed path:
//   1. per-network agreement vs pandapower distributed_slack=True at machine
//      precision — voltages de-referenced (angle differences, so a reference
//      rotation would read as rotation, not physics), and per-generator P;
//   2. a negative control — perturbing a weight makes the distribution diverge
//      from pandapower, so the match is not vacuous;
//   3. a uniform-weight sanity check — equal weights split the imbalance evenly,
//      checkable without the oracle.
final class DistributedSlackTests: XCTestCase {

    private struct Reference: Decodable {
        struct Gen: Decodable {
            var bus: Int; var pSetMw: Double; var vmPu: Double; var slackWeight: Double
        }
        struct Solution: Decodable {
            var vmPu: [Double]; var vaDeg: [Double]; var genPMw: [Double]
        }
        struct Network: Decodable {
            var name: String
            var baseMva: Double
            var buses: [ReferenceCase.RefBus]
            var branches: [ReferenceCase.RefBranch]
            var gens: [Gen]
            var solution: Solution

            /// Build the per-unit network the Swift solver sees, with slack
            /// weights on the generators (which triggers distributed slack).
            func network(perturbGen: Int? = nil, perturbTo: Double = 0) -> BusBranchNetwork {
                let base = baseMva
                let nb = buses.map { b in
                    BusBranchNetwork.Bus(
                        type: BusBranchNetwork.BusType(rawValue: b.type) ?? .pq,
                        baseKv: b.baseKv, pLoadPu: b.pdMw / base, qLoadPu: b.qdMvar / base,
                        gsPu: b.gsMw / base, bsPu: b.bsMvar / base)
                }
                let br = branches.map { b in
                    BusBranchNetwork.Branch(from: b.f, to: b.t, r: b.r, x: b.x, b: b.b, g: b.g,
                                            tap: b.tap <= 0 ? 1.0 : b.tap,
                                            shiftRad: b.shiftDeg * .pi / 180,
                                            inService: b.status == 1)
                }
                let gen = gens.enumerated().map { (k, g) in
                    BusBranchNetwork.Generator(
                        bus: g.bus, pPu: g.pSetMw / base, vSetPu: g.vmPu, vaRefRad: 0,
                        slackWeight: (k == perturbGen) ? perturbTo : g.slackWeight)
                }
                return BusBranchNetwork(baseMVA: base, buses: nb, branches: br, generators: gen)
            }
        }

        var networks: [Network]

        static func load() throws -> Reference {
            guard let url = Bundle.module.url(forResource: "distributed_slack",
                                              withExtension: "json", subdirectory: "Reference") else {
                throw XCTSkip("missing distributed_slack.json — run Tools/dump_reference.py")
            }
            let d = JSONDecoder(); d.keyDecodingStrategy = .convertFromSnakeCase
            return try d.decode(Reference.self, from: Data(contentsOf: url))
        }
    }

    private func options() -> PowerFlowOptions {
        var o = PowerFlowOptions(); o.tolerancePu = 1e-12; return o
    }

    /// De-reference angles against the slack (angle-reference) bus, so a pure
    /// reference rotation cancels instead of reading as a mismatch.
    private func deref(_ va: [Double], ref: Int) -> [Double] { va.map { $0 - va[ref] } }

    // MARK: - 1. Agreement vs pandapower distributed_slack

    func testAgreementVsPandapower() throws {
        let ref = try Reference.load()
        for net in ref.networks {
            let bbn = net.network()
            let sol = NewtonRaphsonSolver().solve(bbn, options: options())
            XCTAssertTrue(sol.converged, "\(net.name): \(sol.failureReason ?? "?")")

            let base = bbn.baseMVA
            let angleRef = bbn.buses.firstIndex { $0.type == .slack }!

            let vm = ErrorStats(reference: net.solution.vmPu, computed: sol.vmPu)
            let refDeg = deref(net.solution.vaDeg.map { $0 * .pi / 180 }, ref: angleRef)
            let gotDeg = deref(sol.vaRad, ref: angleRef)
            let va = ErrorStats(reference: refDeg, computed: gotDeg)
            let gp = ErrorStats(reference: net.solution.genPMw,
                                computed: sol.genPPu.map { $0 * base })

            XCTAssertLessThanOrEqual(vm.maxError, 1e-9, "\(net.name) Vm")
            XCTAssertLessThanOrEqual(va.maxError, 1e-9, "\(net.name) Va (de-referenced)")
            XCTAssertLessThanOrEqual(gp.maxError, 1e-7, "\(net.name) gen P")

            print(String(format: "  %@ distributed slack vs pandapower: max|ΔVm| %.2e pu, "
                         + "max|ΔVa| %.2e rad (de-ref), max|ΔgenP| %.2e MW",
                         net.name, vm.maxError, va.maxError, gp.maxError))
        }
    }

    // MARK: - 2. Negative control — perturbing a weight diverges

    func testNegativeControlPerturbedWeightDiverges() throws {
        let ref = try Reference.load()
        let net = ref.networks.first { $0.name == "grid_two_gen" }!
        let base = net.baseMva

        // Correct weights: matches pandapower.
        let correct = NewtonRaphsonSolver().solve(net.network(), options: options())
        let gpCorrect = ErrorStats(reference: net.solution.genPMw,
                                   computed: correct.genPPu.map { $0 * base })
        XCTAssertLessThanOrEqual(gpCorrect.maxError, 1e-7, "baseline should match")

        // Perturb generator 2's weight (1.0 -> 5.0): the distribution shifts,
        // so it must NO LONGER match pandapower.
        let perturbed = NewtonRaphsonSolver().solve(
            net.network(perturbGen: 2, perturbTo: 5.0), options: options())
        XCTAssertTrue(perturbed.converged)
        let gpPerturbed = ErrorStats(reference: net.solution.genPMw,
                                     computed: perturbed.genPPu.map { $0 * base })
        XCTAssertGreaterThan(gpPerturbed.maxError, 1.0,
                             "perturbed weights must diverge from pandapower (non-vacuous match)")
        print(String(format: "  negative control: correct max|ΔgenP| %.2e MW, "
                     + "perturbed max|ΔgenP| %.2f MW (diverges as required)",
                     gpCorrect.maxError, gpPerturbed.maxError))
    }

    // MARK: - 3. Uniform-weight sanity — even split, independent of pandapower

    func testUniformWeightsSplitEvenly() throws {
        let ref = try Reference.load()
        let net = ref.networks.first { $0.name == "uniform_three" }!
        let bbn = net.network()
        let sol = NewtonRaphsonSolver().solve(bbn, options: options())
        XCTAssertTrue(sol.converged)

        // Each contributor absorbs (solved − setpoint); with equal weights those
        // shares must be equal, to machine precision — no oracle needed.
        let shares = net.gens.enumerated().map { (k, g) in
            sol.genPPu[k] * net.baseMva - g.pSetMw
        }
        let mean = shares.reduce(0, +) / Double(shares.count)
        for (k, s) in shares.enumerated() {
            XCTAssertEqual(s, mean, accuracy: 1e-9,
                           "uniform weights: contributor \(k) share \(s) != mean \(mean)")
        }
        print(String(format: "  uniform weights: %d contributors each absorb %.4f MW "
                     + "(spread %.2e)", shares.count, mean,
                     (shares.max()! - shares.min()!)))
    }
}
