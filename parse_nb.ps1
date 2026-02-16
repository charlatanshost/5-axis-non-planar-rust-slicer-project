$raw = Get-Content "c:\Users\danie\Documents\Non-planar slicer project\s4_notebook.ipynb" -Raw
$nb = $raw | ConvertFrom-Json
$cellNum = 0
foreach($cell in $nb.cells) {
    $cellNum++
    $type = $cell.cell_type
    Write-Host "===== CELL $cellNum [$type] ====="
    foreach($line in $cell.source) {
        Write-Host $line -NoNewline
    }
    Write-Host ""
    Write-Host ""
}
