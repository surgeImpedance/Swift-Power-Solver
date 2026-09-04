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
///
/// EVALUATED ON |V| (D80 seventeenth sitting, "Option A"). A magnitude is a
/// modulus, and the physical load knows nothing of its sign: a
/// constant-current load draws current in proportion to |V|, a
/// constant-impedance one to |V|². Polar Newton updates the magnitude
/// additively and an iterate CAN cross zero; on the signed value the I term
/// then flipped sign — the load injected — and the mismatch equations
/// acquired roots the network does not have (`converged = true` at
/// −0.33 pu, measured, sixteenth sitting). On |V| every load term is even
/// in V and its derivative odd, so the mismatch at (−V, θ) is the mismatch
/// at (V, θ + π) — the same phasor — and a negative iterate is a
/// representation of a physical point, never a different equation. For a
/// positive magnitude the arithmetic is the pre-edit expression bit for
/// bit: |v| is v and the sign factor is exactly 1 (`OptionATests`, and the
/// blast-radius probe over every reference arm). The negative-magnitude
/// guard (`PowerFlowSolution.nonPhysicalMagnitude`) stays: a solution
/// reported in the non-canonical representation is still refused.
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

    /// Load absorbed at bus `i` at magnitude `v`, pu — evaluated on |v|.
    /// Only meaningful when `on`.
    @inline(__always) func p(_ i: Int, _ v: Double) -> Double {
        let m = abs(v)
        return pP[i] + pI[i] * m + pZ[i] * m * m
    }
    @inline(__always) func q(_ i: Int, _ v: Double) -> Double {
        let m = abs(v)
        return qP[i] + qI[i] * m + qZ[i] * m * m
    }
    /// `∂P_L,i/∂V` and `∂Q_L,i/∂V` at magnitude `v` — the Jacobian diagonal
    /// terms: `sign(v)·(pI + 2·pZ·|v|)`, the derivative of the even polynomial.
    @inline(__always) func dPdV(_ i: Int, _ v: Double) -> Double {
        (v < 0 ? -1.0 : 1.0) * (pI[i] + 2 * pZ[i] * abs(v))
    }
    @inline(__always) func dQdV(_ i: Int, _ v: Double) -> Double {
        (v < 0 ? -1.0 : 1.0) * (qI[i] + 2 * qZ[i] * abs(v))
    }
}
