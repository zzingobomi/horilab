# 새 PC 준비

Windows PC에서 이 레포로 작업을 시작하는 순서다. 고른 도구와 이유는 [0008](decisions/0008-environment-and-packages.md)과 [software.md](software.md#개발-환경)에 있다.

## 1. 프로그램 설치

PowerShell에서:

```powershell
winget install --id Git.Git -e
winget install --id astral-sh.uv -e
winget install --id Microsoft.VisualStudioCode -e
```

VS Code 확장:

```powershell
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension charliermarsh.ruff
code --install-extension bernhard-42.ocp-cad-viewer@4.2.0
```

- OCP CAD Viewer 확장은 `uv.lock`의 ocp-vscode와 major.minor 버전을 맞춘다 (지금 4.2).
- 뷰어가 "Quickstart"로 라이브러리를 설치하겠냐고 물으면 하지 않는다. 라이브러리는 uv가 버전을 고정해서 설치한다.
- Claude Code는 VS Code 확장으로 쓴다.

## 2. PC 설정

- Python UTF-8 모드를 켠다. Windows의 기본 인코딩이 cp949라서다.

  ```powershell
  [Environment]::SetEnvironmentVariable('PYTHONUTF8', '1', 'User')
  ```

- VS Code 사용자 설정에 `"python-envs.terminal.autoActivationType": "shellStartup"`을 넣는다. Python Environments 확장이 OCP CAD Viewer를 멈추게 하는 문제를 피한다. PC 단위 설정이라 레포의 `.vscode/settings.json`에는 넣을 수 없다.

## 3. 레포 받기

```powershell
git clone https://github.com/zzingobomi/horilab.git
cd horilab
git log -1 --format="%an <%ae>"
git config user.name "<위에서 본 이름>"
git config user.email "<위에서 본 이메일>"
uv sync
```

- 커밋 작성자는 기존 커밋과 같게, 이 레포에만 설정한다 (`--global` 아님).
- `uv sync`가 Python 3.12와 모든 패키지를 `.venv`에 맞춘다. VS Code는 이 `.venv`를 인터프리터로 쓴다 (`.vscode/settings.json`).

## 4. 확인

[CLAUDE.md](../CLAUDE.md)의 "자주 쓰는 명령"에 있는 커밋 전 검사를 돌려서 모두 통과하는지 본다.

## 5. Claude Code 권한

`.claude/settings.local.json`은 PC마다 두고 git에 넣지 않는다. 이 레포에서 쓰는 설정:

```json
{
  "permissions": {
    "defaultMode": "acceptEdits",
    "allow": ["Bash", "PowerShell", "WebSearch", "WebFetch", "Skill"],
    "ask": [
      "Bash(git push *)", "PowerShell(git push *)",
      "Bash(git reset --hard *)", "PowerShell(git reset --hard *)",
      "Bash(git clean *)", "PowerShell(git clean *)",
      "Bash(git checkout -- *)", "PowerShell(git checkout -- *)",
      "Bash(git restore *)", "PowerShell(git restore *)",
      "Bash(git branch -D *)", "PowerShell(git branch -D *)",
      "Bash(git stash drop *)", "PowerShell(git stash drop *)",
      "Bash(git stash clear *)", "PowerShell(git stash clear *)",
      "Bash(git config --global *)", "PowerShell(git config --global *)",
      "Bash(gh *)", "PowerShell(gh *)",
      "Bash(rm *)", "Bash(rmdir *)",
      "PowerShell(Remove-Item *)", "PowerShell(rm *)", "PowerShell(del *)",
      "PowerShell(erase *)", "PowerShell(rmdir *)", "PowerShell(rd *)", "PowerShell(ri *)",
      "Bash(winget *)", "PowerShell(winget *)",
      "Bash(choco *)", "PowerShell(choco *)",
      "Bash(scoop *)", "PowerShell(scoop *)"
    ]
  }
}
```

- 확인을 받는 것은 push, 되돌리기 어려운 git 명령, 전역 git 설정, 파일 삭제, 패키지 관리자다.
- 웹 검색과 조사는 도구로는 허용돼 있지만, 작업 규칙상 허락을 받은 뒤에만 한다 ([CLAUDE.md](../CLAUDE.md)).

## 6. 개인 자료

`docs-local/`은 git에 없다. 있으면 다른 PC에서 손으로 옮긴다. 없어도 작업은 이어진다.

## 7. 시험 기록 올리고 받기

git에서 뺀 시험 기록은 구글 드라이브로 넘긴다 ([0022](decisions/0022-records-and-sync.md)). PC마다 한 번 로그인한다.

```powershell
uv run tools/sync.py login --client <앱 파일>
```

- 앱 파일은 구글 클라우드에서 만든 OAuth 클라이언트(데스크톱 앱)의 JSON이다. 다른 PC에서 쓴 파일을 옮겨 오거나, 같은 프로젝트에서 클라이언트를 하나 더 만들어 받는다.
- 처음 쓸 때 rclone을 `tools/local/`에 받는다. PC에 설치하지 않는다.
- 브라우저가 열리면 데이터를 둘 구글 계정으로 로그인하고 허용한다. 직접 만든 앱이라 "확인되지 않은 앱" 경고가 나오면 고급 → 이동을 누른다.
- 로그인이 풀리면(권한을 끊었거나 오래 쓰지 않았을 때) `--client` 없이 `login`만 다시 한다.

구글 앱이 아직 없으면 데이터를 둘 계정으로 구글 클라우드 콘솔에서 한 번 만든다 ([rclone 문서](https://rclone.org/drive/#making-your-own-client-id)).

1. 새 프로젝트를 만들고 Google Drive API를 사용 설정한다.
2. Google 인증 플랫폼을 시작한다. 대상은 외부다.
3. 데이터 액세스에 범위 세 개를 넣는다: `https://www.googleapis.com/auth/docs`, `https://www.googleapis.com/auth/drive`, `https://www.googleapis.com/auth/drive.metadata.readonly`.
4. 대상에서 자기 계정을 테스트 사용자로 넣는다.
5. 클라이언트에서 데스크톱 앱 유형으로 만들고, 만들 때 나오는 창에서 JSON을 받는다.
6. 브랜딩에 홈페이지와 개인정보처리방침 주소를 넣고, 그 도메인을 승인된 도메인에 넣는다. 게시하려면 둘 다 있어야 한다. GitHub Pages 같은 무료 페이지면 된다.
7. 대상에서 앱을 게시한다. 테스트 상태로 두면 로그인이 7일마다 풀린다.
