import Foundation

/// Minimal complex double for admittance/voltage arithmetic. Kept local (no
/// external dependency); the hot loops in the solver unpack to re/im arrays
/// for Accelerate anyway.
public struct ComplexD: Equatable, Sendable {
    public var re: Double
    public var im: Double

    public init(_ re: Double, _ im: Double = 0) {
        self.re = re
        self.im = im
    }

    public static let zero = ComplexD(0, 0)

    /// e^{jθ}
    public static func polar(magnitude: Double, angle: Double) -> ComplexD {
        ComplexD(magnitude * cos(angle), magnitude * sin(angle))
    }

    public var conjugate: ComplexD { ComplexD(re, -im) }
    public var magnitude: Double { hypot(re, im) }

    public static func + (a: ComplexD, b: ComplexD) -> ComplexD {
        ComplexD(a.re + b.re, a.im + b.im)
    }
    public static func - (a: ComplexD, b: ComplexD) -> ComplexD {
        ComplexD(a.re - b.re, a.im - b.im)
    }
    public static func * (a: ComplexD, b: ComplexD) -> ComplexD {
        ComplexD(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re)
    }
    public static func * (a: Double, b: ComplexD) -> ComplexD {
        ComplexD(a * b.re, a * b.im)
    }
    public static func / (a: ComplexD, b: ComplexD) -> ComplexD {
        // Smith's algorithm-free version is fine at power-system magnitudes.
        let d = b.re * b.re + b.im * b.im
        return ComplexD((a.re * b.re + a.im * b.im) / d,
                       (a.im * b.re - a.re * b.im) / d)
    }
    public static func / (a: ComplexD, b: Double) -> ComplexD {
        ComplexD(a.re / b, a.im / b)
    }
    public static prefix func - (a: ComplexD) -> ComplexD { ComplexD(-a.re, -a.im) }

    public static func += (a: inout ComplexD, b: ComplexD) { a = a + b }
}
