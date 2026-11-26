# GitHub upload checklist

## Review before the first commit

- [ ] Track `README.md`, `.gitignore`, `docs/*.md`, useful `reports/*.md`, `src/` package manifests/source/launch/config/RViz/tests, `scripts/`, and the original saved `map.yaml` plus `map.pgm`.
- [ ] Include the separate portable Nav2 metadata example; do not replace the original map or existing Nav2 metadata.
- [ ] Keep `build/`, `install/`, `log/`, bytecode/caches, hidden backup directories, secrets, large raw experiment JSON, temporary logs and screenshots out of the commit. Existing local artifacts are preserved.
- [ ] Verify map YAML and PGM SHA256 against the baseline below; on another checkout verify map size 119×79 and 275 unknown cells.
- [ ] Verify ROS 2 Humble, Gazebo Fortress and manifest dependencies via [setup](SETUP_TR.md).
- [ ] Review [README](../README.md), [architecture](ARCHITECTURE.md), [status](PROJECT_STATUS_TR.md), [roadmap](ROADMAP_TR.md), [run guide](RUN_GUIDE_TR.md) and [known issues](KNOWN_ISSUES.md).
- [ ] Scan tracked candidates for credentials and inspect any screenshots proposed for sharing. Never print secret contents in an issue or PR.
- [ ] Review file sizes with `find . -type f -size +20M` and `git status --short --untracked-files=all`. Use Git LFS only if a reviewed artifact genuinely needs versioning.
- [ ] Run `colcon build --symlink-install` and `python3 -m pytest -q src/competition_bringup/test`; record results. No movement test is part of the initial upload.
- [ ] Check `git status`, `git check-ignore -v` on map/source/generated samples, and `git diff --check`. Inspect staged files with `git diff --cached --stat` and `git diff --cached --name-only`.

Map baseline:

```text
27d68b0c7a69a677e61da2cab0a546563cd6c64d9e1012fa4ab6aedfced61abc  maps/competition_mapping/20261125T103259Z_572d1928/map.yaml
1b8051da18f35d084dbcce3dc9fb101110e358c19d805d591dc872ad0c6d6d61  maps/competition_mapping/20261125T103259Z_572d1928/map.pgm
```

## Publish after team review

```bash
git status --short --untracked-files=all
# Stage reviewed paths only, not every untracked file.
git add README.md .gitignore docs src scripts reports/*.md reports/phase5f_protected_baseline.sha256
git add maps/competition_mapping/20261125T103259Z_572d1928/map.yaml maps/competition_mapping/20261125T103259Z_572d1928/map.pgm maps/competition_mapping/20261125T103259Z_572d1928/nav2_portable_example.yaml
git diff --cached --name-only
git diff --cached --check
git commit -m "Document current ROS 2 competition project"
git remote add origin <GitHub-repository-URL>
git push -u origin main
```

Add any other historical map or small evidence file only after individual review. Curated Markdown summaries are intended; raw local artifacts are excluded. If the repository already has history or a remote, adapt the procedure instead of overwriting it. After publishing, give teammates the URL, ask them to clone and follow [setup](SETUP_TR.md), and review access permissions.
