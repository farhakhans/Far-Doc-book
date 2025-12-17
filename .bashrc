# Bash configuration for Git completion

# Enable Git tab completion
if [ -f ~/.git-completion.bash ]; then
  . ~/.git-completion.bash
elif [ -f ./git-completion.bash ]; then
  . ./git-completion.bash
elif [ -f ./.git-completion.bash ]; then
  . ./.git-completion.bash
fi

# Git aliases for common commands
alias g='git'
alias gst='git status'
alias gl='git log --oneline --graph --all'
alias gb='git branch'
alias gc='git checkout'
alias gp='git push origin HEAD'
alias gpl='git pull origin HEAD'
alias ga='git add'
alias gcm='git commit -m'

# Enhanced prompt with Git branch
parse_git_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1)/'
}

# Set the prompt to include Git branch
export PS1="\u@\h:\w \$(parse_git_branch)\$ "

# Enable colored output for Git
export GIT_PS1_SHOWDIRTYSTATE=1
export GIT_PS1_SHOWUNTRACKEDFILES=1
export GIT_PS1_SHOWUPSTREAM="auto"

echo "Git completion and aliases configured"