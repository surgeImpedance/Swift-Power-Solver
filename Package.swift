// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "SwiftPowerSolver",
    platforms: [.macOS(.v14), .iOS(.v17)],
    products: [
        .library(name: "SwiftPowerSolver", targets: ["SwiftPowerSolver"]),
    ],
    targets: [
        .target(name: "SwiftPowerSolver"),
        .testTarget(
            name: "SwiftPowerSolverTests",
            dependencies: ["SwiftPowerSolver"],
            resources: [.copy("Reference"), .copy("FactorsFixtures")]
        ),
    ],
    swiftLanguageModes: [.v5]
)
