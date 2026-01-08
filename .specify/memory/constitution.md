<!-- SYNC IMPACT REPORT
Version change: N/A (initial creation) → 1.0.0
Modified principles: N/A
Added sections: All sections (initial constitution)
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .specify/templates/commands/*.md ⚠ pending review
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Library-First & Modular Thinking
Everything is a module. Each concept MUST be designed as an independent, reusable module. Modules MUST have clear inputs, outputs, and responsibilities. No module may rely on undocumented side effects. Modules SHOULD map to real robotics subsystems (Perception, Planning, Control, Actuation, Simulation). Cross-module coupling MUST be explicit and minimal.

### II. Spec Before Writing
Nothing is written without a specification. Every book element (module, chapter, section) MUST have a written spec before content generation. Specs MUST define: Objective, Scope & non-scope, Key concepts, Inputs / Outputs, Validation criteria. Content generation without an approved spec is forbidden. Specs are the source of truth; prose is a derived artifact.

### III. Deterministic Structure (No Free Writing)
Predictable structure over creative prose. All chapters MUST follow a fixed structure: 1. Problem Definition, 2. System Architecture, 3. Algorithms & Models, 4. Data & Signals, 5. Execution Flow, 6. Failure Modes & Limits. No narrative-only sections are allowed. Opinions, storytelling, and speculative content are prohibited unless explicitly scoped. AI-generated text MUST strictly expand the spec, not reinterpret it.

### IV. Real Robotics Stacks Only
Engineering realism is mandatory. All examples MUST use real-world stacks, including but not limited to: ROS 2 (Nodes, Topics, Services, Actions), Sensors (Cameras, LiDAR, IMU, Force/Torque), Actuators (Servos, Motors, Controllers), AI Models (Perception, World Models, Planning, Control). Hypothetical or fictional hardware/software is not allowed. Each system MUST map clearly to physical hardware or a recognized simulator. Versioning of tools and frameworks SHOULD be stated where relevant.

### V. Reproducibility & Engineering Accuracy
Results must be reproducible. All technical claims MUST be verifiable. Code snippets MUST be minimal, correct, and runnable in principle. Diagrams MUST reflect real data flow and control flow. Assumptions MUST be explicitly stated. Known limitations and failure cases MUST be documented.

### VI. AI Usage Rules (Claude)
AI assists; specs govern. Claude MUST act as a spec executor, not an autonomous author. Claude output MUST: Follow the Constitution, Adhere strictly to provided specs, Avoid assumptions or missing steps. Human review is mandatory before acceptance.

## Quality Gates
Content is accepted ONLY if: Spec compliance is verified, Technical accuracy is reviewed, Structure matches the deterministic template, Reproducibility criteria are met.

## Amendment Policy
This Constitution may only be changed through an explicit, versioned proposal. All downstream content MUST be updated to reflect constitutional changes.

## Governance
This Constitution defines the non-negotiable rules, constraints, and quality standards for developing the Physical AI & Humanoid Robotics Book using SpecKit Plus and AI-assisted authoring. All artifacts (specs, chapters, code, diagrams) MUST comply with this document. All development must follow these principles and undergo compliance verification.

**Version**: 1.0.0 | **Ratified**: 2026-01-07 | **Last Amended**: 2026-01-07