from pathlib import Path
import shutil
import subprocess
import textwrap


WORKFLOW = Path(__file__).parents[2] / ".github" / "workflows" / "upstream-sync.yml"


def workflow_text() -> str:
  return WORKFLOW.read_text()


def shell_block(text: str, start: str, end: str | None = None) -> str:
  block_start = text.index(start)
  block_end = text.index(end, block_start) if end else len(text)
  return textwrap.dedent(text[block_start:block_end])


def test_rewritten_upstream_history_recovery_preserves_custom_safeguards():
  workflow = workflow_text()

  assert 'if ! git merge-base HEAD "$UPSTREAM_SHA"' in workflow
  assert "git merge --allow-unrelated-histories -s ours --no-commit" in workflow
  assert 'git read-tree --reset -u "$UPSTREAM_SHA"' in workflow
  assert "git commit --no-edit" in workflow
  assert "SUNNYLINK_BYPASS_START" in workflow
  assert "driver_distracted = False  # Always attentive" in workflow
  assert "fix_pay_attention" in workflow


def test_normal_ancestry_path_remains_supported():
  workflow = workflow_text()

  assert 'if git merge-base --is-ancestor "$UPSTREAM_SHA" HEAD; then' in workflow
  assert 'git merge "$UPSTREAM_SHA" -m "Merge upstream sunnypilot staging' in workflow


def test_failure_reporting_cannot_mask_the_sync_failure():
  workflow = workflow_text()

  assert "continue-on-error: true" in workflow
  assert "GH_TOKEN: ${{ github.token }}" in workflow
  assert "OPEN_FAILURES=$(gh issue list" in workflow
  assert "printf '%s\\n'" in workflow


def test_workflow_shell_blocks_are_valid_bash():
  if shutil.which("bash") is None:
    return

  workflow = workflow_text()
  blocks = [
    shell_block(workflow, "          set -euo pipefail", "\n\n      - name: Verify custom mods survived"),
    shell_block(workflow, "          set -uo pipefail"),
  ]

  for block in blocks:
    result = subprocess.run(["bash", "-n"], input=block + "\n", text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
