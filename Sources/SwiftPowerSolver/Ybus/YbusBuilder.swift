import Foundation

// Ybus assembly (matpower/pandapower convention):
//
//   ys  = 1 / (r + jx)                series admittance
//   t   = tap · e^{j·shift}           complex ratio at the from side
//   yc  = (g + j·b) / 2               charging admittance per side (pandapower
//                                     carries trafo magnetizing conductance in g)
//   ytt = ys + yc
//   yff = ytt / (t·conj(t))
//   yft = -ys / conj(t)
//   ytf = -ys / t
//
// plus bus shunts gs + j·bs on the diagonal. Out-of-service branches
// contribute nothing.

public enum YbusBuilder {
    /// The four pi-model admittance blocks of one branch. Shared with the
    /// branch-flow calculation so Ybus and flows can never disagree.
    public struct BranchAdmittance: Sendable {
        public let yff: ComplexD
        public let yft: ComplexD
        public let ytf: ComplexD
        public let ytt: ComplexD
    }

    public static func admittance(of branch: BusBranchNetwork.Branch) -> BranchAdmittance {
        let ys = ComplexD(1, 0) / ComplexD(branch.r, branch.x)
        let ytt = ys + ComplexD(branch.g / 2, branch.b / 2)
        let tap = branch.tap
        let t = ComplexD.polar(magnitude: tap, angle: branch.shiftRad)
        return BranchAdmittance(
            yff: ytt / (tap * tap),
            yft: -ys / t.conjugate,
            ytf: -ys / t,
            ytt: ytt)
    }

    public static func build(_ net: BusBranchNetwork) -> SparseComplexMatrix {
        var entries: [(row: Int, col: Int, value: ComplexD)] = []
        entries.reserveCapacity(4 * net.branches.count + net.busCount)
        for branch in net.branches where branch.inService {
            let y = admittance(of: branch)
            let f = branch.from, t = branch.to
            entries.append((f, f, y.yff))
            entries.append((f, t, y.yft))
            entries.append((t, f, y.ytf))
            entries.append((t, t, y.ytt))
        }
        for (i, bus) in net.buses.enumerated() where bus.gsPu != 0 || bus.bsPu != 0 {
            entries.append((i, i, ComplexD(bus.gsPu, bus.bsPu)))
        }
        return SparseComplexMatrix(n: net.busCount, entries: entries)
    }
}
