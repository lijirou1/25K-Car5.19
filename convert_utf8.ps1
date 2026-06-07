$root = "d:\111\25K-Car-副本"

# 获取所有目标文件
$files = @()
$files += Get-ChildItem -Path "$root\Hardware" -Include *.c,*.h | Select-Object -ExpandProperty FullName
$files += Get-ChildItem -Path "$root\system" -Include *.c,*.h | Select-Object -ExpandProperty FullName
$files += Get-ChildItem -Path "$root\user" -Include *.c,*.h | Select-Object -ExpandProperty FullName

$converted = 0
$skipped = 0
$errors = @()

foreach ($f in $files) {
    try {
        $bytes = [System.IO.File]::ReadAllBytes($f)
        $isUtf8 = $false
        if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
            $isUtf8 = $true
        }
        
        $relative = $f.Replace($root + "\", "")
        
        if ($isUtf8) {
            $skipped++
            Write-Host "$relative -> 已是UTF-8，跳过" -ForegroundColor Yellow
        } else {
            # 以ANSI(GBK)编码读取，以UTF-8写入
            $content = [System.IO.File]::ReadAllText($f, [System.Text.Encoding]::Default)
            [System.IO.File]::WriteAllText($f, $content, [System.Text.Encoding]::UTF8)
            $converted++
            Write-Host "$relative -> 已转换为UTF-8" -ForegroundColor Green
        }
    } catch {
        $errors += $f
        $relative = $f.Replace($root + "\", "")
        Write-Host "$relative -> 转换失败: $_" -ForegroundColor Red
    }
}

Write-Host ""
Write-Host "完成！共转换 $converted 个文件，$skipped 个已跳过（已是UTF-8），$($errors.Count) 个失败。" -ForegroundColor Cyan
if ($errors.Count -gt 0) {
    Write-Host "失败文件:" -ForegroundColor Red
    $errors | ForEach-Object { Write-Host $_ -ForegroundColor Red }
}
