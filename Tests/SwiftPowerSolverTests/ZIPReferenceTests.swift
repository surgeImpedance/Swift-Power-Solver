import XCTest
@testable import SwiftPowerSolver

// D80 step 3 — the ZIP (voltage-dependent load) oracle, `Reference/zip.json`,
// and the NON-DROPPABLE anti-vacuity assertions on it (ZIP exploration §4.2):
//
//   (i)  the coefficients are nontrivial — not the (0,0,1) mix that means
//        constant power;
//   (ii) the ZIP solution differs from the constant-power solution of the same
//        case by more than the NR gate tolerance (1e-6 pu);
//   (iii) the constant-power block re-solved in the SAME generation run equals
//        the committed case fixture bit-for-bit — the regeneration control:
//        if the generation options ever drift, this moves before any ZIP
//        answer can be trusted.
//
// There is NO Swift ZIP solve here. The fixture is written before the solver
// on purpose (writing the guarantee is the control). When step 4 lands, the
// comparison test joins this file; these three assertions stay.
//
// `iterations` in the zip blocks is pandapower's PARTIAL-Newton count (its
// Jacobian carries no load term) and is provenance only — never a gate.
struct ZIPReference: Decodable {
    struct Coefficients: Decodable {
        var zP: Double, iP: Double, zQ: Double, iQ: Double, pP: Double, qQ: Double
        var appliedAs: String
        enum CodingKeys: String, CodingKey {
            case zP = "z_p", iP = "i_p", zQ = "z_q", iQ = "i_q", pP = "p_p", qQ = "q_q"
            case appliedAs = "applied_as"
        }
    }
    struct ConsumedLoad: Decodable {
        var bus: [Int]
        var pMw: [Double], qMvar: [Double], scheduledPMw: [Double], scheduledQMvar: [Double]
        enum CodingKeys: String, CodingKey {
            case bus, pMw = "p_mw", qMvar = "q_mvar"
            case scheduledPMw = "scheduled_p_mw", scheduledQMvar = "scheduled_q_mvar"
        }
    }
    struct Solution: Decodable {
        var vmPu: [Double], vaDeg: [Double], genPMw: [Double], genQMvar: [Double]
        var iterations: Int, busType: [Int]
        var consumedLoad: ConsumedLoad?
        enum CodingKeys: String, CodingKey {
            case vmPu = "vm_pu", vaDeg = "va_deg", genPMw = "gen_p_mw", genQMvar = "gen_q_mvar"
            case iterations, busType = "bus_type", consumedLoad = "consumed_load"
        }
    }
    struct Case: Decodable {
        var constantPower: [String: Solution]
        var zip: [String: Solution]
        var maxAbsDvm: [String: Double]
        enum CodingKeys: String, CodingKey {
            case constantPower = "constant_power", zip
            case maxAbsDvm = "max_abs_dvm_zip_minus_constant_power"
        }
    }
    var pandapowerVersion: String
    var coefficients: Coefficients
    var cases: [String: Case]
    enum CodingKeys: String, CodingKey {
        case pandapowerVersion = "pandapower_version", coefficients, cases
    }

    static func load() throws -> ZIPReference {
        let url = try XCTUnwrap(Bundle.module.url(forResource: "zip", withExtension: "json",
                                                  subdirectory: "Reference"))
        return try JSONDecoder().decode(ZIPReference.self, from: Data(contentsOf: url))
    }
}

final class ZIPReferenceTests: XCTestCase {

    private let vmTol = 1e-6   // the NR gate's own tolerance (NewtonRaphsonTests)

    func testCoefficientsAreNontrivialAndSumToOne() throws {
        let c = try ZIPReference.load().coefficients
        XCTAssertGreaterThan(c.zP + c.iP, 0, "P mix is constant power — the fixture is vacuous")
        XCTAssertGreaterThan(c.zQ + c.iQ, 0, "Q mix is constant power — the fixture is vacuous")
        XCTAssertEqual(c.zP + c.iP + c.pP, 1, accuracy: 1e-12)
        XCTAssertEqual(c.zQ + c.iQ + c.qQ, 1, accuracy: 1e-12)
        XCTAssertTrue(c.appliedAs.contains("one load per bus"), "the count-average trap must be excluded by construction")
    }

    func testZIPSolutionDiffersFromConstantPowerBeyondTheGateTolerance() throws {
        let ref = try ZIPReference.load()
        XCTAssertEqual(Set(ref.cases.keys), ["case14", "case39", "case118"])
        for (name, c) in ref.cases {
            for key in ["default", "q_lims"] {
                let cp = try XCTUnwrap(c.constantPower[key], "\(name)/\(key)")
                let z = try XCTUnwrap(c.zip[key], "\(name)/\(key)")
                XCTAssertEqual(cp.vmPu.count, z.vmPu.count, "\(name)/\(key)")
                let dv = zip(cp.vmPu, z.vmPu).map { abs($0 - $1) }.max() ?? 0
                XCTAssertGreaterThan(dv, vmTol, "\(name)/\(key): ZIP ≡ constant power — the fixture is the container, not the content")
                XCTAssertEqual(dv, try XCTUnwrap(c.maxAbsDvm[key]), accuracy: 1e-15, "\(name)/\(key): recorded delta")
                // The consumed load is pandapower's own res_load and must move too.
                let cl = try XCTUnwrap(z.consumedLoad, "\(name)/\(key): consumed load missing")
                let dp = zip(cl.pMw, cl.scheduledPMw).map { abs($0 - $1) }.max() ?? 0
                XCTAssertGreaterThan(dp, 0, "\(name)/\(key): consumed ≡ scheduled under ZIP — vacuous")
                XCTAssertEqual(cl.bus.count, cl.pMw.count)
            }
        }
    }

    /// The regeneration control: the constant-power block re-solved in the
    /// same run as the ZIP block equals the committed fixture bit-for-bit.
    func testConstantPowerControlEqualsCommittedFixtureBitForBit() throws {
        let ref = try ZIPReference.load()
        for name in ["case14", "case39", "case118"] {
            let committed = try ReferenceCase.load(name)
            XCTAssertEqual(ref.pandapowerVersion, committed.pandapowerVersion, name)
            let c = try XCTUnwrap(ref.cases[name])
            let pairs: [(String, ReferenceCase.RefSolution)] = [("default", committed.solutions.plain),
                                                                ("q_lims", committed.solutions.qLims)]
            for (key, sol) in pairs {
                let cp = try XCTUnwrap(c.constantPower[key])
                XCTAssertEqual(cp.vmPu.map(\.bitPattern), sol.vmPu.map(\.bitPattern), "\(name)/\(key) vm")
                XCTAssertEqual(cp.vaDeg.map(\.bitPattern), sol.vaDeg.map(\.bitPattern), "\(name)/\(key) va")
                XCTAssertEqual(cp.genQMvar.map(\.bitPattern), sol.genQMvar.map(\.bitPattern), "\(name)/\(key) genQ")
                XCTAssertEqual(cp.iterations, sol.iterations, "\(name)/\(key) iterations")
            }
        }
    }
}
