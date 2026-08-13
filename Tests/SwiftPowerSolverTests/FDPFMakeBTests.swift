import XCTest
@testable import SwiftPowerSolver

// Gate 1, construction half: the Swift B′/B″ builders must reproduce pypower's
// own makeB matrices (MATPOWER makeB.m semantics) entry for entry, both
// variants, on case14 and case118 — the latter chosen because it exercises
// off-nominal taps, bus shunts, and branch magnetizing g together.

private struct MakeBFixture: Decodable {
    struct Triplets: Decodable {
        var row: [Int]
        var col: [Int]
        var v: [Double]
    }
    struct VariantMatrices: Decodable {
        var bp: Triplets
        var bpp: Triplets
    }
    struct Case: Decodable {
        var name: String
        var n: Int
        var xb: VariantMatrices
        var bx: VariantMatrices
    }
    var pandapowerVersion: String
    var cases: [Case]

    static func load() throws -> MakeBFixture {
        guard let url = Bundle.module.url(forResource: "fdpf_makeb", withExtension: "json",
                                          subdirectory: "Reference") else {
            throw XCTSkip("missing fdpf_makeb.json — run Tools/dump_fdpf_reference.py")
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(MakeBFixture.self, from: Data(contentsOf: url))
    }
}

final class FDPFMakeBTests: XCTestCase {

    /// Dense accumulation of triplets (n ≤ 118, so an n×n array is cheap and
    /// sidesteps any ordering/duplicate-summing differences).
    private func dense(_ n: Int, _ t: [(row: Int, col: Int, value: Double)]) -> [Double] {
        var m = [Double](repeating: 0, count: n * n)
        for e in t { m[e.row * n + e.col] += e.value }
        return m
    }

    private func dense(_ n: Int, _ t: MakeBFixture.Triplets) -> [Double] {
        var m = [Double](repeating: 0, count: n * n)
        for i in t.row.indices { m[t.row[i] * n + t.col[i]] += t.v[i] }
        return m
    }

    func testBMatricesMatchPypowerMakeB() throws {
        let fixture = try MakeBFixture.load()
        for c in fixture.cases {
            let net = try ReferenceCase.load(c.name).network()
            XCTAssertEqual(net.busCount, c.n, "\(c.name): bus count")
            for (variant, ref) in [(FDPFVariant.xb, c.xb), (FDPFVariant.bx, c.bx)] {
                let bp = dense(c.n, FDPFMatrices.bPrimeEntries(net, variant: variant))
                let bpp = dense(c.n, FDPFMatrices.bDoublePrimeEntries(net, variant: variant))
                let refBp = dense(c.n, ref.bp)
                let refBpp = dense(c.n, ref.bpp)
                var worstBp = 0.0, worstBpp = 0.0
                for i in 0..<(c.n * c.n) {
                    worstBp = max(worstBp, abs(bp[i] - refBp[i]))
                    worstBpp = max(worstBpp, abs(bpp[i] - refBpp[i]))
                }
                // Same arithmetic on the same inputs; the bound only covers
                // complex-division rounding differences vs numpy.
                XCTAssertLessThan(worstBp, 1e-10, "\(c.name)/\(variant): B′ vs makeB")
                XCTAssertLessThan(worstBpp, 1e-10, "\(c.name)/\(variant): B″ vs makeB")
                print(String(format: "makeB %@ %@: max|ΔB′|=%.2e, max|ΔB″|=%.2e",
                             c.name, variant.rawValue.uppercased(), worstBp, worstBpp))
            }
        }
    }
}
