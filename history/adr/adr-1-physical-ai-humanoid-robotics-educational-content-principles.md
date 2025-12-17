# ADR-1: Physical AI & Humanoid Robotics Educational Content Principles

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-15
- **Feature:** Physical AI & Humanoid Robotics Book
- **Context:** Establishing core principles for a comprehensive educational book on Physical AI & Humanoid Robotics that bridges digital AI algorithms with physical robot control. The content must be educational, practical, open-source focused, and suitable for academic quarter schedules.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Establish six core principles that will govern all content creation, technical implementation, and educational methodology for the Physical AI & Humanoid Robotics book:

1. **Physical AI & Embodied Intelligence Focus**: All content must bridge the gap between digital AI algorithms and their physical manifestation in humanoid robotics. Every chapter, example, and exercise must connect theoretical AI concepts to practical physical robot control and interaction.

2. **Open-Source Educational Excellence**: Adherence to open-source tools and platforms (ROS 2, Gazebo, Unity, NVIDIA Isaac) is mandatory. Content must be accessible, well-documented, and reproducible by students with standard hardware configurations.

3. **Practical Application Over Theory (NON-NEGOTIABLE)**: Every concept must include hands-on applications with code examples, simulations, and/or real-world implementations. Theory is only acceptable when immediately followed by practical exercises.

4. **Progressive Learning Architecture**: Content must follow a logical progression from basic AI concepts to advanced Physical AI implementations, with clear prerequisites and pacing appropriate for academic schedules.

5. **Documentation & Deployment Standards**: All content must be Docusaurus-compatible with proper navigation, cross-referencing, and GitHub Pages deployment capability. Code examples must include setup instructions and troubleshooting guides.

6. **Assessment & Learning Outcomes Alignment**: Every module must include measurable learning outcomes, assessment methods, and evaluation criteria that support various assessment types.

## Consequences

### Positive

- Clear, consistent educational content aligned with learning objectives
- Open-source focus ensures accessibility and reproducibility for students
- Practical-first approach enhances student engagement and understanding
- Progressive architecture supports effective learning progression
- Proper documentation standards ensure deployable, maintainable content
- Measurable outcomes enable effective assessment and improvement

### Negative

- May limit use of proprietary tools that could offer enhanced functionality
- Practical focus may require more complex examples than theory-only approaches
- Strict documentation requirements add overhead to content creation process
- Assessment alignment may constrain content flexibility

## Alternatives Considered

Alternative 1: Theory-focused approach with minimal practical components
- Why rejected: Would not meet the core goal of bridging digital AI to physical robotics control; students would lack hands-on experience

Alternative 2: Proprietary tools focus with commercial software emphasis
- Why rejected: Would limit accessibility for students and institutions; conflicts with open-source educational excellence principle

Alternative 3: Non-sequential content without progressive learning architecture
- Why rejected: Would create confusion and hinder student comprehension; lacks pedagogical soundness

Alternative 4: Minimal documentation approach with basic deployment
- Why rejected: Would result in poorly maintained content and deployment issues; fails to meet professional standards

## References

- Feature Spec: null
- Implementation Plan: null
- Related ADRs: null
- Evaluator Evidence: history/prompts/constitution/1-create-physical-ai-humanoid.constitution.prompt.md