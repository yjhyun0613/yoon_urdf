# Model Interaction Rules

This document records the user's specific instructions for the AI agent's behavior.

---

## 1. Code Generation Policy
* **Rule**: Do **NOT** modify or create any source code files (Python, XML, C++, etc.) until the user explicitly says **"코드 만들어줘"** (make code).
* **Behavior**: Only discuss the scenario, plan, math, or write markdown documentation/scenarios until that explicit command is given.

## 2. Documentation and History Policy
* **Rule**: Every time a change, problem, method choice, or comparison of alternatives (e.g., Method 1 vs Method 2) occurs, it **MUST** be recorded in detail in the project's technical markdown documentation (e.g., `monocular_3d_mapping_tech.md` or `walkthrough.md`).
* **Behavior**: 
  - Log the specific issue encountered (e.g., boundary distortion/smearing).
  - Document the options explored (e.g., ROI cropping vs. Lens Undistortion + FOV adjustment).
  - Detail the final resolution and performance results.
  - Future AI agents starting a new session **MUST** read this rule file and automatically maintain this documentation logging behavior without requiring repeated user prompts.

## 3. Git Commit Policy
* **Rule**: Keep making git commits steadily/consistently during the project.
* **Behavior**:
  - Whenever a significant feature or task segment is successfully implemented and verified (e.g., creating the calibration node, adding checkerboard geometries), make a git commit with a clear, concise message.
  - Future AI agents **MUST** read this rule file and automatically maintain this git commit behavior.
