---
id: 1
title: Create Physical AI Humanoid Constitution
stage: constitution
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: AA
command: /sp.constitution
labels: [constitution,physical-ai,robotics,humanoid,embodied-intelligence]
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

```text
/sp.constitution

Project: Physical AI & Humanoid Robotics — Embodied Intelligence

Purpose:
Author a spec-driven technical book on Physical AI and humanoid robotics, connecting AI cognition with real-world robotic action. The book must be deployable, reproducible, and interactive.

Audience:
AI, Robotics, and CS students and developers (intermediate–advanced).

Core Principles:
- Physical-world correctness
- AI-to-ROS traceability
- Reproducibility via runnable code
- Modular, spec-driven design
- Zero hallucination tolerance

Authoring & Platform:
- Docs: Docusaurus
- Hosting: GitHub Pages
- Spec Framework: Spec-Kit Plus
- Authoring Tool: Claude Code

Required Stack:
- Robotics: ROS 2, Gazebo, Unity, NVIDIA Isaac
- AI: OpenAI APIs, Whisper, LLM planners
- Backend: FastAPI
- Vector DB: Qdrant Cloud (Free)
- DB: Neon Serverless Postgres

Book Structure:
- Quarter overview + learning outcomes
- 4 modules + capstone
- Each module includes:
  - Concepts
  - Architecture
  - Simulation
  - Code
  - Debugging notes

Modules:
1. ROS 2 Nervous System (nodes, topics, URDF, rclpy)
2. Digital Twin (Gazebo physics, Unity HRI, sensors)
3. AI-Robot Brain (Isaac Sim, VSLAM, Nav2)
4. Vision-Language-Action (Whisper → LLM → ROS)

Capstone:
Autonomous humanoid executing voice commands:
plan → navigate → perceive → manipulate.

RAG Chatbot:
- Embedded in the book
- Uses OpenAI Agents / ChatKit
- Answers from indexed content only
- Supports selected-text Q&A
- Section-level citations required

Quality & Constraints:
- Fully runnable code
- Verifiable claims only
- APA citations where applicable
- Zero plagiarism
- Safety and failure modes documented

Success Criteria:
- Book deploys on GitHub Pages
- Code and simulations run as documented
- RAG chatbot answers accurately without hallucination
- End-to-end VLA capstone demonstrated
```

## Response snapshot

Updated the constitution file with principles and requirements for the Physical AI & Humanoid Robotics project, including core principles like Physical-World Correctness, AI-to-ROS Traceability, Reproducibility via Runnable Code, Modular Design, Zero Hallucination Tolerance, and Interactive Learning Experience. Added technical requirements for robotics stack compliance, AI integration standards, and backend/database requirements. Defined quality assurance and deployment standards for the book project.

## Outcome

- ✅ Impact: Created project constitution with 6 core principles and technical requirements
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: Plan module implementations based on constitution
- 🧠 Reflection: Constitution successfully captures the essence of the Physical AI & Humanoid Robotics project with specific principles aligned to robotics and AI integration requirements.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): Begin module specifications based on constitution