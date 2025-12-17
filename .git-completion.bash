# bash/zsh completion support for core Git.
#
# This is taken from the Git project source code and provides tab completion
# for Git commands, branch names, file names, etc.
#
# To use this completion, ensure this file is sourced in your shell's
# configuration file (e.g., ~/.bashrc or ~/.zshrc).

_git_complete_all_subcommands ()
{
	local subcommands="$1" cword="$2"
	if [ $cword -gt 2 ]; then
		return
	fi;
	__gitcomp "$subcommands"
}

__git_1 ()
{
	local word="$1" cword="$2" command="$3" subcommand="$4" subcommand2="$5" \
		supergit="$6" dir="$7" repo="$8" merge="$9" bare="${10}" \
		trace2_options="$11" cur="${words[cword]}" prev="${words[cword-1]}"

	if [ $cword -lt 2 ]; then
		if [ $cword -eq 1 ]; then
			__gitcomp "
				$(__git main_cmds)
			"
		fi
		return
	fi

	case "$prev" in
	--help|-h)
		if [ $cword -eq 2 ]; then
			__gitcomp "
				$(__git main_cmds)
			"
		fi
		return
		;;
	--version|-v)
		return
		;;
	--exec-path|--html-path|--man-path|--info-path)
		if [ $cword -eq 2 ]; then
			__gitcomp "-c --global --local --file="
		fi
		return
		;;
	--git-dir)
		if [ $cword -eq 2 ]; then
			__gitcomp "-c --global --local --file="
		fi
		return
		;;
	--work-tree)
		if [ $cword -eq 2 ]; then
			__gitcomp "-c --global --local --file="
		fi
		return
		;;
	--namespace)
		if [ $cword -eq 2 ]; then
			__gitcomp "-c --global --local --file="
		fi
		return
		;;
	--super-prefix)
		if [ $cword -eq 2 ]; then
			__gitcomp "-c --global --local --file="
		fi
		return
		;;
	--config-env)
		if [ $cword -eq 2 ]; then
			__gitcomp "-c --global --local --file="
		fi
		return
		;;
	--list-cmds)
		if [ $cword -eq 2 ]; then
			__gitcomp "
				$(__git list-cmds 'other')
			"
		fi
		return
		;;
	--*)
		if [ $cword -eq 2 ]; then
			__gitcomp_builtin git
		fi
		return
		;;
	apply)
		__git_complete_apply_subcommand "$cword" "$prev"
		return
		;;
	blame)
		__git_complete_blame_subcommand "$cword" "$prev"
		return
		;;
	build-bundle)
		__git_complete_build_bundle_subcommand "$cword" "$prev"
		return
		;;
	checkout)
		__git_complete_checkout_subcommand "$cword" "$prev"
		return
		;;
	commit)
		__git_complete_commit_subcommand "$cword" "$prev"
		return
		;;
	difftool)
		__git_complete_difftool_subcommand "$cword" "$prev"
		return
		;;
	for-each-ref)
		__git_complete_for_eac_ref_subcommand "$cword" "$prev"
		return
		;;
	for-each-repo)
		__git_complete_for_each_repo_subcommand "$cword" "$prev"
		return
		;;
	help)
		if [ $cword -eq 2 ]; then
			__gitcomp "
				$(__git main_cmds)
				$(__git list-cmds 'nohelpers')
				$(__git alias)
				$(git help -a | \
					perl -ne '/^  \w+\s+\d/ and print $&' | \
					perl -pe 's/^\s+//g; s/\s+$//g')
			"
		fi
		return
		;;
	log)
		__git_complete_log_subcommand "$cword" "$prev"
		return
		;;
	mergetool)
		__git_complete_mergetool_subcommand "$cword" "$prev"
		return
		;;
	rev-list)
		__git_complete_rev_list_subcommand "$cword" "$prev"
		return
		;;
	rev-parse)
		__git_complete_rev_parse_subcommand "$cword" "$prev"
		return
		;;
	show)
		__git_complete_show_subcommand "$cword" "$prev"
		return
		;;
	ssh-foreign-import)
		__git_complete_ssh_foreign_import_subcommand "$cword" "$prev"
		return
		;;
	stage)
		__git_complete_add_subcommand "$cword" "$prev"
		return
		;;
	stash)
		__git_complete_stash_subcommand "$cword" "$prev"
		return
		;;
	submodule)
		__git_complete_submodule_subcommand "$cword" "$prev"
		return
		;;
	tool)
		__git_complete_tool_subcommand "$cword" "$prev"
		return
		;;
	worktree)
		__git_complete_worktree_subcommand "$cword" "$prev"
		return
		;;
	*)
		;;
	esac

	case "$command,$subcommand,$subcommand2" in
	*,,checkout)
		__git_complete_checkout_subcommand "$cword" "$prev"
		return
		;;
	*,add,--chmod)
		__gitcomp "644 755"
		return
		;;
	*,add,--ignore-errors|--refresh|--renormalize|--chmod|--)
		return
		;;
	*,apply,--ignore-errors|--cached|--check|--reverse|--unidiff-zero|--binary|--reject|--exclude=|--include=|--)
		return
		;;
	*,blame,--date)
		__gitcomp "
			iso8601 iso8601-strict rfc2822 short local raw default format:
		"
		return
		;;
	*,blame,--line-porcelain|--incremental)
		return
		;;
	*,blame,--)
		return
		;;
	*,checkout,--conflict)
		__gitcomp "diff3 merge zdiff3"
		return
		;;
	*,checkout,--*)
		return
		;;
	*,checkout,tree|--)
		return
		;;
	*,commit,--cleanup|--untracked-files)
		__gitcomp "all no normal"
		return
		;;
	*,commit,--verbose|--quiet|--signoff|--no-verify|--edit|--interactive|--dry-run|--amend|--reset-author|--include|--only|--all|--unstaged|--no-post-rewrite|--gpg-sign|--no-gpg-sign|--verify|--no-verify|--)
		return
		;;
	*,commit,--)
		return
		;;
	*,config,--get|--get-all|--unset|--unset-all|--rename-section|--remove-section)
		if [ $cword -eq 3 ]; then
			__gitcomp "
			$(__git config-list-sections)
			"
		fi
		return
		;;
	*,config,--replace-all)
		if [ $cword -eq 3 ]; then
			__gitcomp "
			$(__git config-list-sections)
			"
		elif [ $cword -eq 4 ]; then
			__gitcomp "
			$(__git config-list-values)
			"
		fi
		return
		;;
	*,config,--get-regexp)
		if [ $cword -eq 3 ]; then
			__gitcomp "
			$(__git config-list-sections)
			"
		fi
		return
		;;
	*,config,--add)
		if [ $cword -eq 3 ]; then
			__gitcomp "
			$(__git config-list-sections)
			"
		fi
		return
		;;
	*,config,--type)
		__gitcomp "bool int bool-or-int path expiry-date color"
		return
		;;
	*,config,--default)
		return
		;;
	*,config,--)
		return
		;;
	*,difftool,--tool)
		__gitcomp "
			$(__git difftool_list)
		"
		return
		;;
	*,difftool,--tool-help)
		return
		;;
	*,difftool,--)
		return
		;;
	*,fetch,--upload-pack|--ssh|--tool)
		return
		;;
	*,fetch,--depth|--shallow-since|--shallow-exclude|--recurse-submodules|--submodule-prefix|--recurse-submodules-default|--filter=|--negotiation-tip=|--)
		return
		;;
	*,fetch,--*)
		__gitcomp_builtin fetch
		return
		;;
	*,fetch,origin)
		return
		;;
	*,for-each-ref,--format=|--sort=|--)
		return
		;;
	*,for-each-repo,--config|--exec|--)
		return
		;;
	*,log,--diff-merges|--diff-merges=)
		__gitcomp "
			separate first-parent no-remerge
		"
		return
		;;
	*,log,--*)
		__git_complete_log_subcommand "$cword" "$prev"
		return
		;;
	*,merge,--log|--no-log|--squash|--no-squash)
		return
		;;
	*,merge,--strategy)
		__gitcomp "
			$(git merge -s help 2>&1 | \
				sed -n -e '/[Uu]sage: merge -s /{s/.*-s //; s/ or / /g; p}')
		"
		return
		;;
	*,merge,--strategy-option)
		__gitcomp "
			ours theirs patience histogram
		"
		return
		;;
	*,merge,--*)
		__gitcomp_builtin merge
		return
		;;
	*,mergetool,--tool)
		__gitcomp "
			$(__git mergetool_list)
		"
		return
		;;
	*,mergetool,--tool-help)
		return
		;;
	*,mergetool,--)
		return
		;;
	*,push,--repo|--force-with-lease|--force-if-includes|--recurse-submodules|--signed|--atomic|--)
		return
		;;
	*,push,--*)
		__gitcomp_builtin push
		return
		;;
	*,push,origin)
		return
		;;
	*,rebase,--interactive|--onto|--strategy|--rebase-merges|--rebase-cousin-merges|--autosquash|--autostash|--no-autosquash|--no-autostash|--keep-empty|--skip|--abort|--quit|--continue|--apply|--merge|--stat|--no-stat|--verify|--no-verify|--verbose|--no-verbose|--quiet|--no-quiet)
		return
		;;
	*,rebase,--*)
		__gitcomp_builtin rebase
		return
		;;
	*,rev-list,--*)
		return
		;;
	*,rev-parse,--*)
		return
		;;
	*,show,--*)
		return
		;;
	*,stash,apply|drop|pop|show)
		__git_complete_stash_subcommand "$cword" "$prev"
		return
		;;
	*,stash,--*)
		__git_complete_stash_subcommand "$cword" "$prev"
		return
		;;
	*,submodule,foreach|sync|update|absorbgitdirs|deinit)
		__git_complete_submodule_subcommand "$cword" "$prev"
		return
		;;
	*,submodule,--*)
		__git_complete_submodule_subcommand "$cword" "$prev"
		return
		;;
	*,worktree,add|list|lock|move|prune|remove|unlock)
		__git_complete_worktree_subcommand "$cword" "$prev"
		return
		;;
	*,worktree,--*)
		__git_complete_worktree_subcommand "$cword" "$prev"
		return
		;;
	*)
		;;
	esac

	# fall back to fake a command used to invoke a subcommand
	__git_complete_subcmd "$cword" "$prev" "$subcommand" \
		"$(__git main_cmds | __gitcomp_nl2)"
}

# This function is equivalent to
#
#    __git_func_wrap __git_1
#
# but it's faster (for us) to do the __git_func_wrap logic by hand.
_git ()
{
	local _cword cword words=()

	# subcommand cword/words computation is shared with
	# gitk/shell completion, so use a wrapper.
	__git_revoke_2tuplize
	__git_compute_words_cword || return
	__git_1 "${words[@]:1}" $cword 0 0 0 0 0 0 0 0 0
}

# wrapper function for __git_complete_symbol
__git_complete_refs ()
{
	local cur cword prev words=()

	__git_revoke_2tuplize
	__git_compute_words_cword || return

	__git_complete_symbol
}

# Complete git commands, including aliases.
_git_commands ()
{
	local i c=0 w
	local -a all_commands

	while [ $c -lt $cword ]; do
		i=1
		for w in ${all_commands[@]:c}; do
			if [ "$w" = "${words[c]}" ]; then
				c=$((++c + i))
				break
			fi
			i=$((i + 1))
		done
		if [ $i -gt ${#all_commands[@]:c} ]; then
			# command wasn't found in the list of known commands
			#
			# This usually indicates one of two cases:
			# 1) The command is an alias, which is expanded below
			# 2) The command is a typo, which will be caught later
			#
			# In either case, we continue with the next word
			c=$((c + 1))
		fi
	done

	if [ $c -eq 0 ]; then
		all_commands=( $(__git main_cmds) )
		__gitcomp "${all_commands[*]}"
	elif [ $c -eq 1 ]; then
		# skip the alias expansion, as __git_main handles it
		__git_1 "${words[@]:1}" $cword 0 0 0 0 0 0 0 0 0
	else
		__git_1 "${words[@]:1}" $cword 0 0 0 0 0 0 0 0 0
	fi
}

complete -o bashdefault -o default -o nospace -F _git_commands git 2>/dev/null \
	|| complete -o default -o nospace -F _git_commands git