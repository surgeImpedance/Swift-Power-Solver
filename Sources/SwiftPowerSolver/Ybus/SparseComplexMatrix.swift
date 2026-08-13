import Foundation

/// Square sparse complex matrix in CSR form (row-major), with real and
/// imaginary parts stored as separate arrays so the NR inner loops and any
/// Accelerate calls can consume them directly.
public struct SparseComplexMatrix: Equatable, Sendable {
    public let n: Int
    public let rowStarts: [Int]     // n+1 entries
    public let columns: [Int]       // nnz entries, ascending within a row
    public let re: [Double]
    public let im: [Double]

    public var nonZeroCount: Int { columns.count }

    /// Build from coordinate-form entries; duplicates are summed.
    ///
    /// Assembled with a two-pass LSD radix sort on (column, then row) rather
    /// than the array-of-arrays plus one Dictionary per row this used to build.
    /// Same reasoning as `SparseLinearSolver.compressToCSC`: O(nnz + n) with no
    /// hashing and no per-row allocation.
    ///
    /// BIT-IDENTICAL. Each pass is stable, so entries end up ordered by (row,
    /// column, original order) and each duplicate run is summed in the order it
    /// was supplied — which is exactly the order the Dictionary accumulated in.
    /// That matters here because a bus with parallel branches sums four
    /// contributions per branch, and re-ordering that sum could move the last
    /// bit of a Ybus entry the pandapower comparison checks.
    public init(n: Int, entries: [(row: Int, col: Int, value: ComplexD)]) {
        let count = entries.count
        var rowStarts = [Int](repeating: 0, count: n + 1)
        var columns = [Int]()
        var re = [Double]()
        var im = [Double]()

        if count > 0 {
            // Pass 1 — stable bucket by COLUMN.
            var colStarts = [Int](repeating: 0, count: n + 1)
            for e in entries { colStarts[e.col + 1] += 1 }
            for i in 1...n { colStarts[i] += colStarts[i - 1] }
            var byColRow = [Int](repeating: 0, count: count)
            var byColCol = [Int](repeating: 0, count: count)
            var byColRe = [Double](repeating: 0, count: count)
            var byColIm = [Double](repeating: 0, count: count)
            for e in entries {
                let p = colStarts[e.col]
                byColRow[p] = e.row
                byColCol[p] = e.col
                byColRe[p] = e.value.re
                byColIm[p] = e.value.im
                colStarts[e.col] = p + 1
            }

            // Pass 2 — stable bucket by ROW. Column order survives, so each row
            // comes out ascending by column.
            var bucket = [Int](repeating: 0, count: n + 1)
            for r in byColRow { bucket[r + 1] += 1 }
            for i in 1...n { bucket[i] += bucket[i - 1] }
            let sortedStarts = bucket
            var cursor = bucket
            var sortedCol = [Int](repeating: 0, count: count)
            var sortedRe = [Double](repeating: 0, count: count)
            var sortedIm = [Double](repeating: 0, count: count)
            for k in 0..<count {
                let r = byColRow[k]
                let p = cursor[r]
                sortedCol[p] = byColCol[k]
                sortedRe[p] = byColRe[k]
                sortedIm[p] = byColIm[k]
                cursor[r] = p + 1
            }

            columns.reserveCapacity(count)
            re.reserveCapacity(count)
            im.reserveCapacity(count)
            for row in 0..<n {
                var k = sortedStarts[row]
                let end = sortedStarts[row + 1]
                while k < end {
                    let c = sortedCol[k]
                    var accRe = sortedRe[k], accIm = sortedIm[k]
                    var j = k + 1
                    while j < end && sortedCol[j] == c {
                        accRe += sortedRe[j]; accIm += sortedIm[j]; j += 1
                    }
                    columns.append(c)
                    re.append(accRe)
                    im.append(accIm)
                    k = j
                }
                rowStarts[row + 1] = columns.count
            }
        }

        self.n = n
        self.rowStarts = rowStarts
        self.columns = columns
        self.re = re
        self.im = im
    }

    /// Value at (row, col); zero when the entry is not stored.
    public subscript(row: Int, col: Int) -> ComplexD {
        for k in rowStarts[row]..<rowStarts[row + 1] where columns[k] == col {
            return ComplexD(re[k], im[k])
        }
        return .zero
    }

    /// All stored entries as coordinates (for diffing / debugging).
    public func coordinateEntries() -> [(row: Int, col: Int, value: ComplexD)] {
        var out: [(Int, Int, ComplexD)] = []
        out.reserveCapacity(nonZeroCount)
        for i in 0..<n {
            for k in rowStarts[i]..<rowStarts[i + 1] {
                out.append((i, columns[k], ComplexD(re[k], im[k])))
            }
        }
        return out
    }

    public static func == (a: SparseComplexMatrix, b: SparseComplexMatrix) -> Bool {
        a.n == b.n && a.rowStarts == b.rowStarts && a.columns == b.columns
            && a.re == b.re && a.im == b.im
    }
}
