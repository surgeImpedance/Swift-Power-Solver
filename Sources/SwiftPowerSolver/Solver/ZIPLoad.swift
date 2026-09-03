import Foundation

/// D80 step 4b — the per-bus ZIP load polynomial, captured ONCE at solver
/// entry from the bus components (`Bus.pLoadZPu` …).
///
///     P_L,i(V) = pP[i] + pI[i]·V + pZ[i]·V²        pP = pLoadPu − pZ − pI
///     Q_L,i(V) = qP[i] + qI[i]·V + qZ[i]·V²
///
/// The constant-power remainder `pP` is what carries the reducer's
/// negative-load fold (a folded machine is constant power by definition), so
/// every read of "the load at this bus" through this type treats the fold
/// as constant power — including the Q-limit check and the post-solve
/// reconstructions.
///
/// THE CONTRACT: `on == false` (no bus carries a component) selects the
/// constant-power path UNCHANGED. Every solver reads `on` once at entry and
/// guards every evaluation and every Jacobian term with it; on that path
/// `p(_:_:)` / `q(_:_:)` are never called and the pre-D80 expressions run
/// exactly as before. That is what keeps every constant-power fixture
/// bit-identical (ZIP exploration §3.1, §5.4 item 5).
///
/// The derivative used on the Jacobian diagonal is the UNSCALED `∂/∂|V|`
/// form, `pI + 2·pZ·V`, matching this solver's ΔV unknown (§3.2) — not
/// MATPOWER's `Vm`-scaled form.
struct ZIPLoad {
    let on: Bool
    let pP: [Double], pI: [Double], pZ: [Double]
    let qP: [Double], qI: [Double], qZ: [Double]

    init(_ net: BusBranchNetwork) {
        on = net.hasVoltageDependentLoad
        pZ = net.buses.map(\.pLoadZPu)
        pI = net.buses.map(\.pLoadIPu)
        qZ = net.buses.map(\.qLoadZPu)
        qI = net.buses.map(\.qLoadIPu)
        pP = net.buses.map { $0.pLoadPu - $0.pLoadZPu - $0.pLoadIPu }
        qP = net.buses.map { $0.qLoadPu - $0.qLoadZPu - $0.qLoadIPu }
    }

    /// Load absorbed at bus `i` at magnitude `v`, pu. Only meaningful when `on`.
    @inline(__always) func p(_ i: Int, _ v: Double) -> Double {
        pP[i] + pI[i] * v + pZ[i] * v * v
    }
    @inline(__always) func q(_ i: Int, _ v: Double) -> Double {
        qP[i] + qI[i] * v + qZ[i] * v * v
    }
    /// `∂P_L,i/∂V` and `∂Q_L,i/∂V` at magnitude `v` — the Jacobian diagonal terms.
    @inline(__always) func dPdV(_ i: Int, _ v: Double) -> Double { pI[i] + 2 * pZ[i] * v }
    @inline(__always) func dQdV(_ i: Int, _ v: Double) -> Double { qI[i] + 2 * qZ[i] * v }
}
