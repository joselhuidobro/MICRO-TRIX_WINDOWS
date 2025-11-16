#!/bin/sh
set -eu

GITDIR="/data/.git"
WORKTREE="/repo"
BRANCH="${GIT_BRANCH:-main}"
GLB_REL="${GLB_RELATIVE_PATH:-robot.glb}"
WWW="/www"
METRICS="$WWW/metrics"

git_cmd() { git --git-dir="$GITDIR" --work-tree="$WORKTREE" "$@"; }

sha256_of() {
  # shell portable
  sha256sum "$1" | awk '{print $1}'
}

init_repo() {
  if [ ! -d "$GITDIR" ]; then
    git_cmd init
    git_cmd config core.fileMode false
    git_cmd lfs install || true
    # LFS para .glb (idempotente)
    if [ ! -f "$WORKTREE/.gitattributes" ] || ! grep -q '\*.glb' "$WORKTREE/.gitattributes"; then
      echo "*.glb filter=lfs diff=lfs merge=lfs -text" >> "$WORKTREE/.gitattributes"
    fi
    # ignores básicos
    if [ ! -f "$WORKTREE/.gitignore" ]; then
      printf "%s\n%s\n%s\n" ".DS_Store" "*.tmp" "*.swp" > "$WORKTREE/.gitignore" || true
    fi
    git_cmd add -A
    git_cmd commit -m "Initial import" || true
    git_cmd branch -M "$BRANCH"
  fi
}

write_metrics() {
  mkdir -p "$WWW"
  {
    echo "# HELP git_repo_dirty 1 si hay cambios sin commit"
    echo "# TYPE git_repo_dirty gauge"
    if [ -n "$(git_cmd status --porcelain)" ]; then echo "git_repo_dirty 1"; else echo "git_repo_dirty 0"; fi

    BR=$(git_cmd rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)
    echo "# HELP git_repo_branch Rama actual"
    echo "# TYPE git_repo_branch gauge"
    echo "git_repo_branch{branch=\"${BR}\"} 1"

    CT=$(git_cmd show -s --format=%ct HEAD 2>/dev/null || echo 0)
    echo "# HELP git_repo_head_epoch Epoch del último commit"
    echo "# TYPE git_repo_head_epoch gauge"
    echo "git_repo_head_epoch ${CT}"

    P="${WORKTREE}/${GLB_REL}"
    echo "# HELP glb_file_size_bytes Tamaño del GLB"
    echo "# TYPE glb_file_size_bytes gauge"
    echo "# HELP glb_file_mtime_epoch mtime del GLB (epoch)"
    echo "# TYPE glb_file_mtime_epoch gauge"
    echo "# HELP glb_file_info Etiqueta con hash del GLB"
    echo "# TYPE glb_file_info gauge"
    if [ -f "$P" ]; then
      SIZE=$(stat -c '%s' "$P" 2>/dev/null || echo 0)
      MTIME=$(stat -c '%Y' "$P" 2>/dev/null || echo 0)
      HASH=$(sha256_of "$P")
      echo "glb_file_size_bytes{path=\"${GLB_REL}\"} ${SIZE}"
      echo "glb_file_mtime_epoch{path=\"${GLB_REL}\"} ${MTIME}"
      echo "glb_file_info{path=\"${GLB_REL}\",sha256=\"${HASH}\"} 1"
    fi
  } > "${METRICS}.tmp"
  mv -f "${METRICS}.tmp" "$METRICS"
}

start_watcher() {
  # auto-commit y refresco de métricas
  inotifywait -m -r -e close_write,create,move,delete --format "%e %w%f" "$WORKTREE" \
  | while read EVT FILE; do
      case "$FILE" in *~|*.swp|*.tmp|*/.git/*) continue;; esac
      git_cmd add -A
      if [ -n "$(git_cmd status --porcelain)" ]; then
        MSG="Auto-commit: $(date -Iseconds) EVT=$EVT FILE=$(basename "$FILE")"
        if [ -f "$FILE" ] && echo "$FILE" | grep -qiE '\.glb$'; then
          SIZE=$(stat -c '%s' "$FILE" 2>/dev/null || echo 0)
          MTIME_H=$(stat -c '%y' "$FILE" 2>/dev/null || echo "")
          HASH=$(sha256_of "$FILE")
          MSG="$MSG SIZE=$SIZE MTIME=\"$MTIME_H\" SHA256=$HASH"
        fi
        git_cmd commit -m "$MSG" || true
      fi
      write_metrics
    done &
}

main() {
  init_repo
  write_metrics
  start_watcher
  # refresco periódico por si cambian tiempos sin inotify
  ( while true; do sleep 10; write_metrics; done ) &
  # servidor HTTP mínimo (BusyBox httpd)
  
  exec /usr/sbin/httpd -f -p 7011 -h "$WWW"

}

main "$@"

