import Foundation
import XCTest
@testable import SwiftPowerSolver

// Codable mirror of tools/dump_reference.py output — the pandapower oracle.

struct ReferenceCase: Decodable {
    struct RefBus: Decodable {
        var i: Int
        var type: Int
        var pdMw: Double
        var qdMvar: Double
        var gsMw: Double
        var bsMvar: Double
        var baseKv: Double
    }

    struct RefBranch: Decodable {
        var f: Int
        var t: Int
        var r: Double
        var x: Double
        var b: Double
        var g: Double
        var tap: Double
        var shiftDeg: Double
        var status: Int
    }

    struct RefGen: Decodable {
        var bus: Int
        var pgMw: Double
        var qmaxMvar: Double
        var qminMvar: Double
        var vgPu: Double
        var vaDeg: Double
        var status: Int
    }

    struct RefYbus: Decodable {
        var n: Int
        var row: [Int]
        var col: [Int]
        var g: [Double]
        var b: [Double]
    }

    struct RefFlow: Decodable {
        var pfMw: Double
        var qfMvar: Double
        var ptMw: Double
        var qtMvar: Double
    }

    struct RefSolution: Decodable {
        var vmPu: [Double]
        var vaDeg: [Double]
        var branchFlows: [RefFlow]
        var genPMw: [Double]
        var genQMvar: [Double]
        var iterations: Int
        var busType: [Int]
    }

    struct Solutions: Decodable {
        var plain: RefSolution
        var qLims: RefSolution

        // Explicit keys: convertFromSnakeCase would otherwise also rewrite
        // dictionary keys, and `default` is a keyword anyway.
        enum CodingKeys: String, CodingKey {
            case plain = "default"
            case qLims = "qLims"   // post-snake-case conversion of "q_lims"
        }
    }

    // Piece 2 oracle: pandapower rundcpp results. Optional so pre-DC reference
    // JSONs (and the AC-only tests) keep decoding unchanged.
    struct RefDC: Decodable {
        var vaDeg: [Double]
        var branchPFromMw: [Double]
        var branchPToMw: [Double]
    }

    // Piece 3 oracle: pandapower makeLODF columns + post-contingency DC flows
    // from a real re-solve. Optional, same additive pattern as `dc`.
    struct RefContingency: Decodable {
        struct Outage: Decodable {
            var branch: Int
            var islands: Bool
            /// Column k of pandapower's LODF; nil for islanding outages, where
            /// pypower emits inf/nan (see DistributionFactors.isIslanding).
            var lodfColumn: [Double?]?
            /// Post-contingency branch P from a rundcpp re-solve; entries are
            /// nil where pandapower reports NaN (buses stranded by the outage).
            var postPFromMw: [Double?]
        }
        var islandingBranches: [Int]
        var outages: [Outage]
    }

    var name: String
    var pandapowerVersion: String
    var baseMva: Double
    var buses: [RefBus]
    var branches: [RefBranch]
    var gens: [RefGen]
    var ybus: RefYbus
    var solutions: Solutions
    var dc: RefDC?
    var contingency: RefContingency?

    /// Load `<name>.json` from the bundled Reference directory.
    static func load(_ name: String) throws -> ReferenceCase {
        guard let url = Bundle.module.url(forResource: name, withExtension: "json",
                                          subdirectory: "Reference") else {
            throw XCTSkip("missing reference \(name).json — run Tools/dump_reference.py")
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(ReferenceCase.self, from: Data(contentsOf: url))
    }

    /// The per-unit network the Swift solver sees (same data pandapower fed
    /// its own Ybus/NR, so any output difference is a Swift bug).
    func network() -> BusBranchNetwork {
        let base = baseMva
        let nb = buses.map { b in
            BusBranchNetwork.Bus(
                type: BusBranchNetwork.BusType(rawValue: b.type) ?? .pq,
                baseKv: b.baseKv,
                pLoadPu: b.pdMw / base,
                qLoadPu: b.qdMvar / base,
                gsPu: b.gsMw / base,
                bsPu: b.bsMvar / base)
        }
        let br = branches.map { b in
            BusBranchNetwork.Branch(
                from: b.f, to: b.t, r: b.r, x: b.x, b: b.b, g: b.g,
                tap: b.tap <= 0 ? 1.0 : b.tap,   // matpower: tap 0 means nominal
                shiftRad: b.shiftDeg * .pi / 180,
                inService: b.status == 1)
        }
        let gen = gens.map { g in
            BusBranchNetwork.Generator(
                bus: g.bus, pPu: g.pgMw / base, vSetPu: g.vgPu,
                vaRefRad: g.vaDeg * .pi / 180,
                qMinPu: g.qminMvar / base, qMaxPu: g.qmaxMvar / base,
                inService: g.status == 1)
        }
        return BusBranchNetwork(baseMVA: base, buses: nb, branches: br, generators: gen)
    }
}

/// Max/mean absolute error over paired values, with the worst index —
/// the shared shape of every "diff against pandapower" report.
struct ErrorStats {
    var maxError = 0.0
    var meanError = 0.0
    var worstIndex = -1

    init(reference: [Double], computed: [Double]) {
        precondition(reference.count == computed.count)
        var sum = 0.0
        for (i, (r, c)) in zip(reference, computed).enumerated() {
            let e = abs(r - c)
            sum += e
            if e > maxError { maxError = e; worstIndex = i }
        }
        meanError = reference.isEmpty ? 0 : sum / Double(reference.count)
    }

    func description(_ quantity: String, unit: String) -> String {
        String(format: "%@: max %.3e %@ (at %d), mean %.3e %@",
               quantity, maxError, unit, worstIndex, meanError, unit)
    }
}
