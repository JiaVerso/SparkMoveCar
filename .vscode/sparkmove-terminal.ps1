function global:prompt {
    $lastExitCode = $global:LASTEXITCODE
    $location = $executionContext.SessionState.Path.CurrentLocation

    Write-Host 'PS ' -NoNewline -ForegroundColor Magenta
    Write-Host $location -NoNewline -ForegroundColor Cyan
    Write-Host '>' -NoNewline -ForegroundColor Yellow
    Write-Host ' ' -NoNewline

    $global:LASTEXITCODE = $lastExitCode
    return ''
}
