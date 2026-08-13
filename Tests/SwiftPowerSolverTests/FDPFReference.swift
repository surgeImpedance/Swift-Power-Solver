import Foundation
import XCTest
@testable import SwiftPowerSolver

// Slim pandapower oracle for the FDPF gates: ppc-level parameters plus the
// plain (no Q-limits) solved voltages — no Ybus triplets, no contingency
// dump. Produced by Tools/dump_fdpf_reference.py, which exists so the larger
// FDPF gate cases (case30, case300) don't pay the full dump_reference.py
// price in checked-in bytes. Field shapes are identical to ReferenceCase's,
// so the row types are reused.

struct FDPFReferenceCase: Decodable {
    struct SlimSolution: Decodable {
        var vmPu: [Double]
        var vaDeg: [Double]
        var iterations: Int
    }

    var name: String
    var pandapowerVersion: String
    var baseMva: Double
    var buses: [ReferenceCase.RefBus]
    var branches: [ReferenceCase.RefBranch]
    var gens: [ReferenceCase.RefGen]
    var solution: SlimSolution

    /// Load `fdpf_<name>.json` from the bundled Reference directory.
    static func load(_ name: String) throws -> FDPFReferenceCase {
        guard let url = Bundle.module.url(forResource: "fdpf_\(name)", withExtension: "json",
                                          subdirectory: "Reference") else {
            throw XCTSkip("missing reference fdpf_\(name).json — run Tools/dump_fdpf_reference.py")
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(FDPFReferenceCase.self, from: Data(contentsOf: url))
    }

    /// Same per-unit conversion as `ReferenceCase.network()`.
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
                tap: b.tap <= 0 ? 1.0 : b.tap,
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
