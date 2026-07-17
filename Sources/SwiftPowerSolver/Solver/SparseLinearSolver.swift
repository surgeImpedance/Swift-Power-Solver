import Foundation
import Accelerate

/// Sparse direct solve of A·x = b for a square, generally unsymmetric A
/// (the NR Jacobian), via Accelerate's Sparse Solvers with a QR
/// factorization (LU is not offered; QR handles unsymmetric systems).
enum SparseLinearSolver {

    /// `entries` are coordinate-form (row, col, value); duplicates are summed.
    /// Returns nil if the factorization fails (structurally or numerically
    /// singular Jacobian).
    static func solve(n: Int,
                      entries: [(row: Int, col: Int, value: Double)],
                      rhs: [Double]) -> [Double]? {
        precondition(rhs.count == n)

        // Coordinate -> CSC with duplicate summing.
        var byColumn = [[Int: Double]](repeating: [:], count: n)
        for e in entries {
            byColumn[e.col][e.row, default: 0] += e.value
        }
        var columnStarts = [Int](); columnStarts.reserveCapacity(n + 1)
        var rowIndices = [Int32]()
        var values = [Double]()
        columnStarts.append(0)
        for c in 0..<n {
            for r in byColumn[c].keys.sorted() {
                rowIndices.append(Int32(r))
                values.append(byColumn[c][r]!)
            }
            columnStarts.append(rowIndices.count)
        }

        var x = [Double](repeating: 0, count: n)
        var b = rhs
        var ok = false

        columnStarts.withUnsafeMutableBufferPointer { cs in
            rowIndices.withUnsafeMutableBufferPointer { ri in
                values.withUnsafeMutableBufferPointer { vals in
                    let structure = SparseMatrixStructure(
                        rowCount: Int32(n), columnCount: Int32(n),
                        columnStarts: cs.baseAddress!, rowIndices: ri.baseAddress!,
                        attributes: SparseAttributes_t(), blockSize: 1)
                    let matrix = SparseMatrix_Double(structure: structure,
                                                     data: vals.baseAddress!)
                    let factorization = SparseFactor(SparseFactorizationQR, matrix)
                    defer { SparseCleanup(factorization) }
                    guard factorization.status == SparseStatusOK else { return }

                    b.withUnsafeMutableBufferPointer { bp in
                        x.withUnsafeMutableBufferPointer { xp in
                            let bVec = DenseVector_Double(count: Int32(n),
                                                          data: bp.baseAddress!)
                            let xVec = DenseVector_Double(count: Int32(n),
                                                          data: xp.baseAddress!)
                            SparseSolve(factorization, bVec, xVec)
                        }
                    }
                    ok = true
                }
            }
        }

        guard ok, x.allSatisfy({ $0.isFinite }) else { return nil }
        return x
    }
}
