# PowerShell profile for Git completion
# This file sets up Git command completion in PowerShell

# Function to enable Git tab completion in PowerShell
if (Get-Module -ListAvailable -Name posh-git) {
    Import-Module posh-git
    Write-Host "posh-git module loaded for Git tab completion" -ForegroundColor Green
} else {
    Write-Host "posh-git module not found. Install it with: Install-Module posh-git -Scope CurrentUser" -ForegroundColor Yellow
    Write-Host "Or install with: PowerShell -ExecutionPolicy Bypass -Command Install-Module posh-git -Scope CurrentUser" -ForegroundColor Yellow
}

# Git aliases for common commands
Set-Alias g git
Set-Alias gst git status
Set-Alias gl "git log --oneline --graph --all"
Set-Alias gb git branch
Set-Alias gc git checkout
Set-Alias gp "git push origin HEAD"
Set-Alias gpl "git pull origin HEAD"

# Function to show git status in prompt
function Write-Prompt {
    $realLASTEXITCODE = $LASTEXITCODE
    $prompt = ""

    # Add current directory
    $prompt += "$(Get-Location) "

    # Add git branch if in a git repo
    $gitBranch = $(git symbolic-ref --short HEAD 2>$null)
    if ($gitBranch) {
        $prompt += "[$gitBranch] "
    }

    # Add exit code
    $prompt += "`n$realLASTEXITCODE PS> "

    return $prompt
}

# Set the prompt
function prompt {
    Write-Prompt
}

Write-Host "Git completion and aliases configured for PowerShell" -ForegroundColor Green
Write-Host "Available aliases: g, gst, gl, gb, gc, gp, gpl" -ForegroundColor Cyan