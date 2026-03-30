$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

Write-Host 'Repo:' $PWD.Path
Write-Host ''
$msg = Read-Host 'Commit message (supports Chinese)'
if ([string]::IsNullOrWhiteSpace($msg)) {
    Write-Host 'Commit message is empty.'
    exit 1
}

Write-Host ''
Write-Host '[1/5] Status'
git status --short
if ($LASTEXITCODE -ne 0) {
    Write-Host 'git status failed.'
    exit 1
}

$pending = git status --porcelain
if ($LASTEXITCODE -ne 0) {
    Write-Host 'git status --porcelain failed.'
    exit 1
}
if ([string]::IsNullOrWhiteSpace(($pending | Out-String))) {
    Write-Host 'No changes to commit.'
    exit 1
}

Write-Host ''
Write-Host '[2/5] Add'
git add .
if ($LASTEXITCODE -ne 0) {
    Write-Host 'git add failed.'
    exit 1
}

Write-Host ''
Write-Host '[3/5] Commit'
git commit -m $msg
if ($LASTEXITCODE -ne 0) {
    Write-Host 'git commit failed.'
    exit 1
}

Write-Host ''
Write-Host '[4/5] Push'
git push
if ($LASTEXITCODE -ne 0) {
    Write-Host 'git push failed.'
    exit 1
}

Write-Host ''
Write-Host '[5/5] Done'
exit 0
