---
id: 001
title: amend-constitution
stage: constitution
date: 2026-01-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: sp.constitution
labels: ["constitution", "setup", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - ".specify/memory/constitution.md"
 - ".specify/templates/plan-template.md"
 - ".specify/templates/spec-template.md"
 - ".specify/templates/tasks-template.md"
tests:
 - null
---

## Prompt

```text
# Physical AI & Humanoid Robotics Book — Constitution

## Purpose

This Constitution defines the non-negotiable rules, constraints, and quality standards for developing the **Physical AI & Humanoid Robotics Book** using **SpecKit Plus** and AI-assisted authoring. All artifacts (specs, chapters, code, diagrams) MUST comply with this document.

---

## I. Library-First & Modular Thinking

**Principle:** Everything is a module.

* Each concept MUST be designed as an independent, reusable module.
* Modules MUST have clear inputs, outputs, and responsibilities.
* No module may rely on undocumented side effects.
* Modules SHOULD map to real robotics subsystems (Perception, Planning, Control, Actuation, Simulation).
* Cross-module coupling MUST be explicit and minimal.

---

## II. Spec Before Writing

**Principle:** Nothing is written without a specification.

* Every book element (module, chapter, section) MUST have a written spec before content generation.
* Specs MUST define:

  * Objective
  * Scope & non-scope
  * Key concepts
  * Inputs / Outputs
  * Validation criteria
* Content generation without an approved spec is forbidden.
* Specs are the source of truth; prose is a derived artifact.

---

## III. Deterministic Structure (No Free Writing)

**Principle:** Predictable structure over creative prose.

* All chapters MUST follow a fixed structure:

  1. Problem Definition
  2. System Architecture
  3. Algorithms & Models
  4. Data & Signals
  5. Execution Flow
  6. Failure Modes & Limits
* No narrative-only sections are allowed.
* Opinions, storytelling, and speculative content are prohibited unless explicitly scoped.
* AI-generated text MUST strictly expand the spec, not reinterpret it.

---

## IV. Real Robotics Stacks Only

**Principle:** Engineering realism is mandatory.

* All examples MUST use real-world stacks, including but not limited to:

  * ROS 2 (Nodes, Topics, Services, Actions)
  * Sensors (Cameras, LiDAR, IMU, Force/Torque)
  * Actuators (Servos, Motors, Controllers)
  * AI Models (Perception, World Models, Planning, Control)
* Hypothetical or fictional hardware/software is not allowed.
* Each system MUST map clearly to physical hardware or a recognized simulator.
* Versioning of tools and frameworks SHOULD be stated where relevant.

---

## V. Reproducibility & Engineering Accuracy

**Principle:** Results must be reproducible.

* All technical claims MUST be verifiable.
* Code snippets MUST be minimal, correct, and runnable in principle.
* Diagrams MUST reflect real data flow and control flow.
* Assumptions MUST be explicitly stated.
* Known limitations and failure cases MUST be documented.

---

## VI. AI Usage Rules (Claude)

**Principle:** AI assists; specs govern.

* Claude MUST act as a spec executor, not an autonomous author.
* Claude output MUST:

  * Follow the Constitution
  * Adhere strictly to provided specs
  * Avoid assumptions or missing steps
* Human review is mandatory before acceptance.

---

## VII. Quality Gates

Content is accepted ONLY if:

* Spec compliance is verified
* Technical accuracy is reviewed
* Structure matches the deterministic template
* Reproducibility criteria are met

---

## VIII. Amendment Policy

* This Constitution may only be changed through an explicit, versioned proposal.
* All downstream content MUST be updated to reflect constitutional changes.

---

**This Constitution is binding for the entire lifecycle of the Physical AI & Humanoid Robotics Book project.**
```

## Response snapshot

I've created the Physical AI & Humanoid Robotics Book Constitution based on your input, replacing the placeholder template. The constitution has been placed in `.specify/memory/constitution.md` and several templates have been updated to align with the new principles:

1. Created the constitution file with all the specified principles and governance structure
2. Updated the plan template to include constitution checks aligned with the new principles
3. Updated the spec template to include the required elements from the constitution
4. Updated the tasks template to emphasize constitution compliance

The constitution establishes six core principles:
- Library-First & Modular Thinking
- Spec Before Writing
- Deterministic Structure
- Real Robotics Stacks Only
- Reproducibility & Engineering Accuracy
- AI Usage Rules

It also includes Quality Gates, Amendment Policy, and Governance sections as specified.

## Outcome

- ✅ Impact: Created project constitution and updated templates to align with principles
- 🧪 Tests: No tests required for this configuration task
- 📁 Files: Updated .specify/memory/constitution.md, .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
- 🔁 Next prompts: Ready for feature specification and development following the new constitution
- 🧠 Reflection: The constitution establishes a solid foundation for the Physical AI & Humanoid Robotics Book project with clear principles for modular, spec-driven development using real robotics stacks.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable for this configuration task
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin specifying first feature according to the new constitution