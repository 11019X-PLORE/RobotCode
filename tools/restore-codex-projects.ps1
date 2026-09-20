$ErrorActionPreference = "Stop"

$codexDir = Join-Path $env:USERPROFILE ".codex"
$statePath = Join-Path $codexDir ".codex-global-state.json"
$backupDir = Join-Path $codexDir "recovery-backups"
New-Item -ItemType Directory -Force -Path $backupDir | Out-Null

$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
Copy-Item -LiteralPath $statePath -Destination (Join-Path $backupDir ".codex-global-state.before-project-restore.$stamp.json") -Force

Get-Process -ErrorAction SilentlyContinue |
  Where-Object { $_.ProcessName -in @("ChatGPT", "codex", "codex-code-mode-host", "node_repl") } |
  Stop-Process -Force

Start-Sleep -Seconds 3

$pythonExe = Join-Path $env:USERPROFILE ".cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
$script = @'
import json
import os
import time
import uuid
from pathlib import Path

state_path = Path(os.environ["USERPROFILE"]) / ".codex" / ".codex-global-state.json"
state = json.loads(state_path.read_text(encoding="utf-8"))
projects = state.setdefault("local-projects", {})
order = state.setdefault("project-order", [])
now_ms = int(time.time() * 1000)

wanted = [
    ("FRC OffSeasonRobotCode2026", [r"F:\FRCcode\26-code\OffSeasonRobotCode2026"]),
    ("Photonvision2026", [r"F:\FRCcode\26-code\Photonvision2026"]),
    ("Ultralytics", [r"F:\FRCcode\26-code\ultralytics"]),
    ("FTC\u5b66\u4e60", [r"F:\FTCCode\2026Code"]),
    ("FTC\u5b66\u4e60-ControlHub", [r"F:\FTCCode\2026Code\FtcMecanumControlHub"]),
    ("\u6c42\u804c\u6750\u6599", [r"F:\u5c31\u4e1a\u6c42\u804c\u6750\u6599"]),
    ("FRC\u4e0b\u534a\u5e74\u8be6\u7ec6\u8ba1\u5212\u4e0e\u5b89\u6392", [r"C:\Users\允悦\Documents\FRC\u4e0b\u534a\u5e74\u8be6\u7ec6\u8ba1\u5212\u4e0e\u5b89\u6392"]),
    ("\u516c\u4f17\u53f7-\u8d35\u5dde\u901a\u5929\u6cb3\u6f02\u6d41\u6e38", [r"F:\u516c\u4f17\u53f7\u8d35\u5dde\u901a\u5929\u6cb3\u6f02\u6d41\u6e38"]),
]

existing_by_path = {}
for pid, project in projects.items():
    for root in project.get("rootPaths", []):
        existing_by_path[root.lower()] = pid

for name, roots in wanted:
    pid = None
    for root in roots:
        pid = existing_by_path.get(root.lower())
        if pid:
            break

    if pid is None:
        pid = str(uuid.uuid4())
        projects[pid] = {
            "id": pid,
            "name": name,
            "rootPaths": roots,
            "createdAt": now_ms,
            "updatedAt": now_ms,
        }
    else:
        projects[pid]["name"] = name
        projects[pid]["rootPaths"] = roots
        projects[pid]["updatedAt"] = now_ms

    if pid not in order:
        order.append(pid)

state["project-order"] = [pid for pid in order if pid in projects]

persisted = state.setdefault("electron-persisted-atom-state", {})
for pid in state["project-order"]:
    persisted[f"sidebar-project-expanded-v1-codex:{pid}"] = True

state["selected-project"] = {
    "type": "local",
    "projectId": next(
        pid for pid, project in projects.items()
        if project.get("name") == "FRC OffSeasonRobotCode2026"
    ),
}

state_path.write_text(json.dumps(state, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
print("Restored", len(projects), "local projects.")
'@

if (Test-Path -LiteralPath $pythonExe) {
  $script | & $pythonExe -
} else {
  throw "Bundled Python not found: $pythonExe"
}

Write-Host ""
Write-Host "Codex project records restored."
Write-Host "Now reopen Codex / ChatGPT from the Start menu."
