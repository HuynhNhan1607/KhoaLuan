# Build script for multiple robots from single source code (Windows PowerShell)
# Usage: .\build_robots.ps1 [robot_id]
#   - No args: build both robot1 and robot2
#   - With arg: build only specified robot (1 or 2)

$ErrorActionPreference = "Stop"

function Build-Robot {
    param (
        [int]$id
    )
    Write-Host "==========================================" -ForegroundColor Cyan
    Write-Host "  Building Robot $id" -ForegroundColor Cyan
    Write-Host "==========================================" -ForegroundColor Cyan

    idf.py -B "build_robot$id" "-DROBOT_ID=$id" build

    if ($LASTEXITCODE -ne 0) {
        throw "Build failed for Robot $id"
    }

    # Copy binary to project root
    Copy-Item -Path "build_robot$id/robot$id.bin" -Destination "./robot$id.bin" -Force

    # Get file size
    $binFile = Get-Item "./robot$id.bin"
    $sizeKb = [math]::Round($binFile.Length / 1KB, 2)
    Write-Host "Success: robot$id.bin created (Size: $sizeKb KB)" -ForegroundColor Green

    # Show ESP-IDF memory size summary
    Write-Host "Memory usage summary for Robot ${id}:" -ForegroundColor Yellow
    idf.py -B "build_robot$id" size
}

# Check if id is specified
if ($args.Count -eq 0) {
    # Build both
    Build-Robot 1
    Build-Robot 2
    Write-Host ""
    Write-Host "==========================================" -ForegroundColor Green
    Write-Host "  Build Complete!" -ForegroundColor Green
    Write-Host "==========================================" -ForegroundColor Green
    Write-Host "Output files:"
    Write-Host "  - robot1.bin"
    Write-Host "  - robot2.bin"
} else {
    $id = [int]$args[0]
    Build-Robot $id
}
