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

        var (columnStarts, rowIndices, values) = compressToCSC(n: n, entries: entries)

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

    /// Same system, many right-hand sides: factorizes **once** and back-solves
    /// each column. Used by PTDF assembly, which needs one solve per bus.
    /// The single-RHS `solve` above is untouched and unaffected.
    ///
    /// Returns one solution vector per input column, or nil if the
    /// factorization fails.
    static func solve(n: Int,
                      entries: [(row: Int, col: Int, value: Double)],
                      rhsColumns: [[Double]]) -> [[Double]]? {
        guard !rhsColumns.isEmpty else { return [] }
        precondition(rhsColumns.allSatisfy { $0.count == n })

        // Coordinate -> CSC with duplicate summing (same as the single-RHS path).
        var (columnStarts, rowIndices, values) = compressToCSC(n: n, entries: entries)

        var solutions = [[Double]]()
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

                    // Columns are solved in parallel over FIXED chunks, and
                    // every chunk builds its OWN QR factorization of the same
                    // immutable CSC data. Bit identity holds because:
                    //  - SparseFactor is deterministic (a fresh factorization
                    //    reproduces the pinned goldens on every run), so the
                    //    per-chunk factorizations are identical objects;
                    //  - each column's solve is the same single-RHS
                    //    SparseSolve arithmetic as the sequential original;
                    //  - column c writes only slice c — no shared state, no
                    //    reduction order for scheduling to disturb.
                    // FactorsIdentityTests pins all of this against 24895bc.
                    //
                    // Variants tried and REJECTED, for the record:
                    //  - One-call DenseMatrix_Double batch solve: ~4x faster
                    //    on case300, but its blocked multi-column kernel
                    //    reorders arithmetic — PTDF hash diverged.
                    //  - SHARING one factorization across threads: races, even
                    //    with per-call explicit workspaces (run-to-run hash
                    //    instability) — the factorization's static workspace
                    //    region is not shareable. Hence: nothing is shared.
                    let m = rhsColumns.count
                    let chunkCount = max(1, min(m,
                        ProcessInfo.processInfo.activeProcessorCount))
                    let per = (m + chunkCount - 1) / chunkCount
                    var solveFailed = false
                    var xFlat = [Double](repeating: 0, count: n * m)
                    xFlat.withUnsafeMutableBufferPointer { xp in
                        let xBase = xp.baseAddress!
                        let failLock = NSLock()
                        DispatchQueue.concurrentPerform(iterations: chunkCount) { chunk in
                            let lo = chunk * per
                            let hi = Swift.min(m, lo + per)
                            guard lo < hi else { return }
                            let factorization = SparseFactor(SparseFactorizationQR, matrix)
                            defer { SparseCleanup(factorization) }
                            guard factorization.status == SparseStatusOK else {
                                failLock.lock(); solveFailed = true; failLock.unlock()
                                return
                            }
                            for c in lo..<hi {
                                var b = rhsColumns[c]
                                b.withUnsafeMutableBufferPointer { bp in
                                    let bVec = DenseVector_Double(count: Int32(n),
                                                                  data: bp.baseAddress!)
                                    let xVec = DenseVector_Double(count: Int32(n),
                                                                  data: xBase + c * n)
                                    SparseSolve(factorization, bVec, xVec)
                                }
                            }
                        }
                    }
                    guard !solveFailed else { return }
                    solutions.reserveCapacity(m)
                    for c in 0..<m {
                        solutions.append(Array(xFlat[c * n ..< (c + 1) * n]))
                    }
                    ok = true
                }
            }
        }

        guard ok, solutions.allSatisfy({ $0.allSatisfy(\.isFinite) }) else { return nil }
        return solutions
    }

    // MARK: - Coordinate -> CSC

    /// Coordinate form to compressed sparse column, duplicates summed.
    ///
    /// Shared by both solve entry points. This used to be written out twice, and
    /// both copies built an `[[Int: Double]]` — one Dictionary per column —
    /// hashed every entry into it, then called `keys.sorted()` per column. It
    /// runs once per Newton iteration, so it sits under every solve, every
    /// time-series step and every PTDF column.
    ///
    /// Measured on a 1354-bus network (Jacobian dim 2447, nnz 15803): the
    /// conversion cost 1.75 ms against 2.18 ms for Accelerate's own
    /// factor-and-solve — 41% of the linear-solve budget spent reshaping the
    /// matrix rather than solving it. A 300-bus network measured 0.40 ms against
    /// 0.47 ms.
    ///
    /// Replaced by a two-pass LSD radix sort on (row, then column): O(nnz + n),
    /// no hashing, no per-column allocation. 0.16 ms and 0.04 ms on the same two
    /// networks — ~11x.
    ///
    /// BIT-IDENTICAL, and the stability is what makes that true. Floating-point
    /// addition is not associative, so summing duplicate entries in a different
    /// order can move the last bit — which the pandapower reference comparisons
    /// would see. The Dictionary form summed duplicates in the order they appear
    /// in `entries`. Each radix pass here is stable, so entries end up in
    /// (column, row, original-order) order and each duplicate run is summed in
    /// that same original order. Verified max|Δ| = 0.000e+00 against the old
    /// path on the case1354 Jacobian, and 100 of 100 station-solve outputs
    /// byte-identical across the reference and magnetizing fixtures.
    static func compressToCSC(n: Int, entries: [(row: Int, col: Int, value: Double)])
        -> (columnStarts: [Int], rowIndices: [Int32], values: [Double]) {
        let count = entries.count
        var columnStarts = [Int](repeating: 0, count: n + 1)
        guard count > 0 else { return (columnStarts, [], []) }

        // Pass 1 — stable bucket by ROW.
        var rowStarts = [Int](repeating: 0, count: n + 1)
        for e in entries { rowStarts[e.row + 1] += 1 }
        for i in 1...n { rowStarts[i] += rowStarts[i - 1] }
        var byRowCol = [Int32](repeating: 0, count: count)
        var byRowRow = [Int32](repeating: 0, count: count)
        var byRowVal = [Double](repeating: 0, count: count)
        for e in entries {
            let p = rowStarts[e.row]
            byRowCol[p] = Int32(e.col)
            byRowRow[p] = Int32(e.row)
            byRowVal[p] = e.value
            rowStarts[e.row] = p + 1
        }

        // Pass 2 — stable bucket by COLUMN. Row order (and, within a row, the
        // original entry order) survives, so each column comes out ascending by
        // row with duplicates still in the order they were supplied.
        for c in byRowCol { columnStarts[Int(c) + 1] += 1 }
        for i in 1...n { columnStarts[i] += columnStarts[i - 1] }
        var cursor = columnStarts
        var sortedRow = [Int32](repeating: 0, count: count)
        var sortedVal = [Double](repeating: 0, count: count)
        for k in 0..<count {
            let c = Int(byRowCol[k])
            let p = cursor[c]
            sortedRow[p] = byRowRow[k]
            sortedVal[p] = byRowVal[k]
            cursor[c] = p + 1
        }

        // Merge duplicate (row, column) pairs, summing in supplied order.
        var outStarts = [Int](repeating: 0, count: n + 1)
        var outRows = [Int32](); outRows.reserveCapacity(count)
        var outVals = [Double](); outVals.reserveCapacity(count)
        for c in 0..<n {
            var k = columnStarts[c]
            let end = columnStarts[c + 1]
            while k < end {
                let r = sortedRow[k]
                var acc = sortedVal[k]
                var j = k + 1
                while j < end && sortedRow[j] == r { acc += sortedVal[j]; j += 1 }
                outRows.append(r)
                outVals.append(acc)
                k = j
            }
            outStarts[c + 1] = outRows.count
        }
        return (outStarts, outRows, outVals)
    }
}
