---
id: 1
title: Create Physical AI Humanoid Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution,physical-ai,humanoid-robotics,education]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution

Project: Create a comprehensive book on "Physical AI & Humanoid Robotics" using Docusaurus for documentation structure and deploy it to GitHub Pages. Use Spec-Kit Plus for spec-driven development and Claude Code for AI-assisted content generation.

Core Principles:
- Focus on AI Systems in the Physical World and Embodied Intelligence.
- Goal: Bridge the digital brain and physical body, enabling students to apply AI knowledge to control humanoid robots in simulated and real-world environments.
- Structure: Follow the provided quarter overview, including modules on ROS 2, Gazebo & Unity, NVIDIA Isaac, and Vision-Language-Action (VLA).
- Theme: Emphasize the transition from digital AI to embodied intelligence, with learning outcomes, weekly breakdown, assessments, hardware requirements, and lab architecture options.
- Output Format: Generate Docusaurus-compatible MD  files for chapters, sidebar navigation, and configuration. Ensure the book is educational, practical, and includes code examples, diagrams, and deployment instructions.
- Constraints: Adhere to open-source tools; ensure content is accurate, up-to-date as of December 15, 2025; prioritize clarity and accessibility for students; avoid proprietary details without citations.
- Success Criteria: The book should be deployable to GitHub Pages, fully navigable, and cover all outlined sections without omission

## Response snapshot

Created a comprehensive constitution for the Physical AI & Humanoid Robotics book project. The constitution establishes six core principles focusing on:
1. Physical AI & Embodied Intelligence Focus
2. Open-Source Educational Excellence
3. Practical Application Over Theory (NON-NEGOTIABLE)
4. Progressive Learning Architecture
5. Documentation & Deployment Standards
6. Assessment & Learning Outcomes Alignment

Additional sections cover Technical Constraints, Educational Methodology, and Governance. The document ensures all content will be Docusaurus-compatible, open-source focused, practically applicable, and educationally effective.

## Outcome

- ✅ Impact: Created foundational constitution document that will guide all future development of the Physical AI & Humanoid Robotics book
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Proceed with creating spec document for the book content
- 🧠 Reflection: The constitution establishes clear guidelines for content creation, ensuring consistency with educational goals and technical requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - Constitution properly created with all required sections
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Create the initial book specification based on this constitution