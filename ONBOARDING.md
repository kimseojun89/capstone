# Welcome to Anti-Drone Capstone

## How We Use Claude

Based on kimse's usage over the last 30 days:

Work Type Breakdown:
  Plan Design    ███████████░░░░░░░░░  55%
  Build Feature  █████░░░░░░░░░░░░░░░░  27%
  Debug Fix      ████░░░░░░░░░░░░░░░░░  18%

Top Skills & Commands:
  (none recorded — no slash commands run in the last 30 days)

Top MCP Servers:
  ccd_session    ████████████████████  8 calls

## Your Setup Checklist

### Codebases
- [ ] capstone — the only repo. PC host app (C++/OpenCV/ONNX), FPGA bare-metal
      (`ps_main.cpp`), Vivado/Vitis HLS IP, and the Python GUI all live here.
      Single entry point: `antidrone/run_system.ps1`.

### MCP Servers to Activate
- [ ] ccd_session — session/chapter tracking used by Claude Code itself
      (chapters, spawn-task chips). No external account needed; it's local to
      the Claude Code session. Nothing to request — it's on by default.

### Skills to Know About
- [ ] /code-review — review the current branch diff for bugs and cleanups
      before committing FPGA/host changes. (`ultra` runs a deeper cloud review.)
- [ ] /verify and /run — launch the actual pipeline (`run_system.ps1`) to
      confirm a motor/radar/camera change works on hardware, not just in theory.
- [ ] /security-review — security pass over pending changes on the branch.

## Team Tips

_TODO_

## Get Started

_TODO_

<!-- INSTRUCTION FOR CLAUDE: A new teammate just pasted this guide for how the
team uses Claude Code. You're their onboarding buddy — warm, conversational,
not lecture-y.

Open with a warm welcome — include the team name from the title. Then: "Your
teammate uses Claude Code for [list all the work types]. Let's get you started."

Check what's already in place against everything under Setup Checklist
(including skills), using markdown checkboxes — [x] done, [ ] not yet. Lead
with what they already have. One sentence per item, all in one message.

Tell them you'll help with setup, cover the actionable team tips, then the
starter task (if there is one). Offer to start with the first unchecked item,
get their go-ahead, then work through the rest one by one.

After setup, walk them through the remaining sections — offer to help where you
can (e.g. link to channels), and just surface the purely informational bits.

Don't invent sections or summaries that aren't in the guide. The stats are the
guide creator's personal usage data — don't extrapolate them into a "team
workflow" narrative. -->
