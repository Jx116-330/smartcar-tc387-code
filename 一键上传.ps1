$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

Write-Host "当前仓库：$PWD"
Write-Host ""
$msg = Read-Host '请输入本次更新说明（支持中文）'
if ([string]::IsNullOrWhiteSpace($msg)) {
    Write-Host '更新说明不能为空。'
    exit 1
}

Write-Host ""
Write-Host '[1/4] 查看仓库状态...'
git status --short
if ($LASTEXITCODE -ne 0) { throw 'git status 执行失败' }

Write-Host ""
Write-Host '[2/4] 添加变更...'
git add .
if ($LASTEXITCODE -ne 0) { throw 'git add 执行失败' }

Write-Host ""
Write-Host '[3/4] 提交变更...'
git commit -m "$msg"
if ($LASTEXITCODE -ne 0) {
    Write-Host ''
    Write-Host '提交失败。可能是没有可提交的变更。'
    exit 1
}

Write-Host ""
Write-Host '[4/4] 推送到 GitHub...'
git push
if ($LASTEXITCODE -ne 0) { throw 'git push 执行失败' }

Write-Host ""
Write-Host '上传完成。'
exit 0
