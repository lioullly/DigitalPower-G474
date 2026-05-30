---
name: no-auto-flash
description: User wants build only, no auto flash after code changes
metadata:
  type: feedback
---

Default to build only (cmake), do NOT flash after code changes unless the user explicitly asks to flash.

**Why:** User prefers to flash manually at their own timing, not automatically after every edit.

**How to apply:** After any code edit, always run cmake build and report the result. Only run pyocd flash when user explicitly says "烧录" or "flash".
