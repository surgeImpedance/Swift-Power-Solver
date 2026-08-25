import Foundation
import XCTest
@testable import SwiftPowerSolver

/// D6: is D18's residency guardrail a control or a hope?
///
/// D18 estimates factors residency from network dimensions with
/// `(3n² + nbr·n + nbr²) × 8`, pinned at 5,296,539,624 B (4.93 GiB) for
/// case9241. A guardrail that UNDER-estimates admits a case that then jetsams,
/// so the estimate has to be checked against something real.
///
/// WHY `maximum resident set size` IS THE WRONG INSTRUMENT, and why an earlier
/// reading of 4.23 GB must not be compared to the model: macOS compresses
/// memory, and `time -l`'s max RSS excludes compressed pages, so it can sit far
/// below the allocated footprint. It is also process-wide — the earlier figure
/// came from a run that ALSO allocates a ~2.06 GB hashing buffer.
///
/// `phys_footprint` from `TASK_VM_INFO` is the right one: it is what iOS jetsam
/// accounts against, compressed pages included. Sampled on a background thread
/// so the transient peak during the build is caught, not just the retained box.
final class FootprintTests: XCTestCase {

    private func note(_ m: String) {
        FileHandle.standardError.write(Data((m + "\n").utf8))
    }

    static func physFootprint() -> Int {
        var info = task_vm_info_data_t()
        var count = mach_msg_type_number_t(MemoryLayout<task_vm_info_data_t>.size
                                           / MemoryLayout<integer_t>.size)
        let kerr = withUnsafeMutablePointer(to: &info) {
            $0.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
                task_info(mach_task_self_, task_flavor_t(TASK_VM_INFO), $0, &count)
            }
        }
        return kerr == KERN_SUCCESS ? Int(info.phys_footprint) : -1
    }

    private static func gib(_ b: Int) -> String {
        String(format: "%.3f GiB", Double(b) / 1_073_741_824)
    }

    func testCase9241FactorsFootprint() throws {
        guard let paths = ProcessInfo.processInfo.environment["SPS_FACTORS_CASES"],
              let path = paths.split(separator: ",").map(String.init)
                  .first(where: { $0.contains("9241") }) else {
            XCTFail("SPS_FACTORS_CASES must include the case9241 fixture for D6.")
            return
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let fixture = try decoder.decode(Net.self,
                                         from: Data(contentsOf: URL(fileURLWithPath: path)))
        let net = fixture.network()
        let n = net.busCount, nbr = net.branches.count

        let model = (3 * n * n + nbr * n + nbr * nbr) * 8
        let retainedModel = (nbr * n + nbr * nbr) * 8

        let baseline = Self.physFootprint()
        let peak = UnsafeMutablePointer<Int>.allocate(capacity: 1)
        peak.pointee = baseline
        let stop = UnsafeMutablePointer<Bool>.allocate(capacity: 1)
        stop.pointee = false
        defer { peak.deallocate(); stop.deallocate() }

        let sampler = Thread {
            while !stop.pointee {
                peak.pointee = max(peak.pointee, Self.physFootprint())
                usleep(2000)                      // 2 ms
            }
        }
        sampler.start()

        let t0 = ContinuousClock.now
        let factors = DistributionFactors.build(net)
        let dt = ContinuousClock.now - t0
        let afterBuild = Self.physFootprint()
        stop.pointee = true
        Thread.sleep(forTimeInterval: 0.02)
        let observedPeak = peak.pointee

        let ms = Double(dt.components.seconds) * 1000
               + Double(dt.components.attoseconds) / 1e15

        note("D6 case9241 (n=\(n), nbr=\(nbr)) build \(String(format: "%.0f", ms)) ms")
        note("D6   baseline phys_footprint  : \(Self.gib(baseline))")
        note("D6   peak during build        : \(Self.gib(observedPeak)) "
             + "(delta \(Self.gib(observedPeak - baseline)))")
        note("D6   retained after build     : \(Self.gib(afterBuild)) "
             + "(delta \(Self.gib(afterBuild - baseline)))")
        note("D6   D18 model (3n²+nbr·n+nbr²): \(Self.gib(model)) — the guardrail's estimate")
        note("D6   model, retained terms only: \(Self.gib(retainedModel)) (nbr·n + nbr²)")
        note(String(format: "D6   estimate / measured PEAK  : %.2fx",
                    Double(model) / Double(observedPeak - baseline)))
        note(String(format: "D6   estimate / measured RETAIN: %.2fx",
                    Double(model) / Double(afterBuild - baseline)))
        note("D6   VERDICT: the guardrail must OVER-estimate to be a control; "
             + "a ratio above 1.0 is the safe direction.")

        XCTAssertGreaterThan(factors.branchCount, 0)
        XCTAssertGreaterThan(observedPeak, baseline, "the sampler caught nothing")
    }

    private struct Net: Decodable {
        struct B: Decodable { var i: Int; var type: Int; var pdMw, qdMvar, gsMw, bsMvar, baseKv: Double }
        struct R: Decodable { var f, t: Int; var r, x, b, g, tap, shiftDeg: Double; var status: Int }
        struct G: Decodable { var bus: Int; var pgMw, qmaxMvar, qminMvar, vgPu, vaDeg: Double; var status: Int }
        var name: String
        var baseMva: Double
        var buses: [B]
        var branches: [R]
        var gens: [G]

        func network() -> BusBranchNetwork {
            BusBranchNetwork(
                baseMVA: baseMva,
                buses: buses.map {
                    BusBranchNetwork.Bus(type: BusBranchNetwork.BusType(rawValue: $0.type) ?? .pq,
                                         baseKv: $0.baseKv,
                                         pLoadPu: $0.pdMw / baseMva, qLoadPu: $0.qdMvar / baseMva,
                                         gsPu: $0.gsMw / baseMva, bsPu: $0.bsMvar / baseMva)
                },
                branches: branches.map {
                    BusBranchNetwork.Branch(from: $0.f, to: $0.t, r: $0.r, x: $0.x, b: $0.b, g: $0.g,
                                            tap: $0.tap <= 0 ? 1.0 : $0.tap,
                                            shiftRad: $0.shiftDeg * .pi / 180,
                                            inService: $0.status == 1)
                },
                generators: gens.map {
                    BusBranchNetwork.Generator(bus: $0.bus, pPu: $0.pgMw / baseMva, vSetPu: $0.vgPu,
                                               vaRefRad: $0.vaDeg * .pi / 180,
                                               qMinPu: $0.qminMvar / baseMva,
                                               qMaxPu: $0.qmaxMvar / baseMva,
                                               inService: $0.status == 1)
                })
        }
    }
}
