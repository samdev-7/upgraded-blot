// remove_bg.swift — isolate the foreground of an image using Apple's Vision
// framework (`VNGenerateForegroundInstanceMaskRequest`, macOS 14+).
//
// Writes two files:
//   <output.png>       — foreground composited onto pure white (for the
//                        grayscale/stipple pipeline — background = no ink)
//   <output>.mask.png  — single-channel mask (255 = foreground, 0 = background)
//
// Usage:
//   swift remove_bg.swift <input> <output.png>
//
// Exit codes: 0 on success (including "no foreground detected" — the input is
// copied through unchanged and a fully-foreground mask is written so downstream
// code that relies on the mask keeps working).

import Foundation
import Vision
import CoreImage
import ImageIO
import UniformTypeIdentifiers

func die(_ msg: String, code: Int32 = 1) -> Never {
    FileHandle.standardError.write(Data((msg + "\n").utf8))
    exit(code)
}

func maskPath(for outPath: String) -> String {
    let url = URL(fileURLWithPath: outPath)
    let stem = url.deletingPathExtension().lastPathComponent
    return url.deletingLastPathComponent()
        .appendingPathComponent("\(stem).mask.png").path
}

func writePNG(_ cgImage: CGImage, to path: String) {
    let url = URL(fileURLWithPath: path)
    guard let dest = CGImageDestinationCreateWithURL(url as CFURL,
                                                      UTType.png.identifier as CFString,
                                                      1, nil) else {
        die("failed to open output destination: \(path)")
    }
    CGImageDestinationAddImage(dest, cgImage, nil)
    guard CGImageDestinationFinalize(dest) else {
        die("failed to finalize output: \(path)")
    }
}

func solidMaskCGImage(width: Int, height: Int) -> CGImage? {
    let cs = CGColorSpaceCreateDeviceGray()
    guard let ctx = CGContext(data: nil, width: width, height: height,
                              bitsPerComponent: 8, bytesPerRow: width,
                              space: cs,
                              bitmapInfo: CGImageAlphaInfo.none.rawValue) else {
        return nil
    }
    ctx.setFillColor(CGColor(gray: 1.0, alpha: 1.0))
    ctx.fill(CGRect(x: 0, y: 0, width: width, height: height))
    return ctx.makeImage()
}

guard CommandLine.arguments.count == 3 else {
    die("usage: remove_bg <input> <output.png>")
}
let inPath = CommandLine.arguments[1]
let outPath = CommandLine.arguments[2]
let outMaskPath = maskPath(for: outPath)

let inURL = URL(fileURLWithPath: inPath)
guard let src = CGImageSourceCreateWithURL(inURL as CFURL, nil),
      let cgImage = CGImageSourceCreateImageAtIndex(src, 0, nil) else {
    die("could not load image: \(inPath)")
}

let handler = VNImageRequestHandler(cgImage: cgImage, options: [:])
let req = VNGenerateForegroundInstanceMaskRequest()

do {
    try handler.perform([req])
} catch {
    die("vision request failed: \(error)")
}

guard let obs = req.results?.first else {
    // No foreground detected — copy the original and emit a full-foreground mask.
    FileHandle.standardError.write(Data("no foreground detected; passing through\n".utf8))
    try? FileManager.default.removeItem(atPath: outPath)
    try? FileManager.default.copyItem(atPath: inPath, toPath: outPath)
    if let allFg = solidMaskCGImage(width: cgImage.width, height: cgImage.height) {
        writePNG(allFg, to: outMaskPath)
    }
    exit(0)
}

// --- composited foreground on white ------------------------------------------
let maskedBuffer: CVPixelBuffer
do {
    maskedBuffer = try obs.generateMaskedImage(
        ofInstances: obs.allInstances,
        from: handler,
        croppedToInstancesExtent: false)
} catch {
    die("masked image generation failed: \(error)")
}

let masked = CIImage(cvPixelBuffer: maskedBuffer)
let whiteBG = CIImage(color: CIColor.white).cropped(to: masked.extent)
let composited = masked.composited(over: whiteBG)

let ctx = CIContext()
guard let outCG = ctx.createCGImage(composited, from: composited.extent) else {
    die("failed to render composited CGImage")
}
writePNG(outCG, to: outPath)

// --- raw foreground mask as a single-channel PNG -----------------------------
let maskPixelBuffer: CVPixelBuffer
do {
    maskPixelBuffer = try obs.generateScaledMaskForImage(
        forInstances: obs.allInstances,
        from: handler)
} catch {
    die("mask generation failed: \(error)")
}

// The mask buffer is a single-channel float; render it as grayscale at full
// image resolution so it aligns pixel-for-pixel with the composited output.
let maskCI = CIImage(cvPixelBuffer: maskPixelBuffer)
let targetExtent = composited.extent
let sx = targetExtent.width / maskCI.extent.width
let sy = targetExtent.height / maskCI.extent.height
let scaledMask = maskCI.transformed(by: CGAffineTransform(scaleX: sx, y: sy))
guard let maskCG = ctx.createCGImage(scaledMask, from: targetExtent) else {
    die("failed to render mask CGImage")
}
writePNG(maskCG, to: outMaskPath)
