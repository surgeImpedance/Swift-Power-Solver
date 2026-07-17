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
    public init(n: Int, entries: [(row: Int, col: Int, value: ComplexD)]) {
        var byRow: [[(col: Int, value: ComplexD)]] = Array(repeating: [], count: n)
        for e in entries {
            byRow[e.row].append((e.col, e.value))
        }
        var rowStarts = [Int](); rowStarts.reserveCapacity(n + 1)
        var columns = [Int]()
        var re = [Double]()
        var im = [Double]()
        rowStarts.append(0)
        for row in byRow {
            // Sum duplicates, then emit in column order.
            var acc: [Int: ComplexD] = [:]
            for (c, v) in row { acc[c, default: .zero] += v }
            for c in acc.keys.sorted() {
                let v = acc[c]!
                columns.append(c)
                re.append(v.re)
                im.append(v.im)
            }
            rowStarts.append(columns.count)
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
