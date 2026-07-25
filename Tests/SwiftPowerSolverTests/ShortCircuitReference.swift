import Foundation
import XCTest
@testable import SwiftPowerSolver

// Codable mirror of Tools/dump_reference.py's shortcircuit.json — purpose-built
// IEC 60909 reference networks (the IEEE cases carry no short-circuit data).
// Each network is authored in per-unit and fed to pandapower in the equivalent
// engineering units, so both sides solve identical data.

struct ShortCircuitReference: Decodable {
    struct RefBus: Decodable { var baseKv: Double; var type: Int }
    struct RefBranch: Decodable { var f: Int; var t: Int; var rPu: Double; var xPu: Double }
    struct RefSource: Decodable { var bus: Int; var scRPu: Double; var scXPu: Double }

    struct RefFault: Decodable {
        var ikssKa: Double
        var ipKa: Double?
        var ithKa: Double?
        var rkOhm: Double
        var xkOhm: Double
        var skssMva: Double?
    }

    struct Network: Decodable {
        var name: String
        var baseMva: Double
        var c: Double
        var tkS: Double
        var freqHz: Double
        var buses: [RefBus]
        var branches: [RefBranch]
        var sources: [RefSource]
        var results: [String: [RefFault?]]   // "3ph" / "2ph" -> per bus, null = edge

        /// The per-unit network the Swift short-circuit solver sees.
        func network() -> BusBranchNetwork {
            let nb = buses.map {
                BusBranchNetwork.Bus(
                    type: BusBranchNetwork.BusType(rawValue: $0.type) ?? .pq,
                    baseKv: $0.baseKv)
            }
            let br = branches.map {
                BusBranchNetwork.Branch(from: $0.f, to: $0.t, r: $0.rPu, x: $0.xPu)
            }
            let gen = sources.map {
                BusBranchNetwork.Generator(
                    bus: $0.bus, pPu: 0, vSetPu: 1.0,
                    scSubtransientRPu: $0.scRPu, scSubtransientXPu: $0.scXPu)
            }
            return BusBranchNetwork(baseMVA: baseMva, buses: nb, branches: br,
                                    generators: gen)
        }
    }

    var pandapowerVersion: String
    var networks: [Network]

    static func load() throws -> ShortCircuitReference {
        guard let url = Bundle.module.url(forResource: "shortcircuit",
                                          withExtension: "json",
                                          subdirectory: "Reference") else {
            throw XCTSkip("missing shortcircuit.json — run Tools/dump_reference.py")
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(ShortCircuitReference.self,
                                  from: Data(contentsOf: url))
    }
}
