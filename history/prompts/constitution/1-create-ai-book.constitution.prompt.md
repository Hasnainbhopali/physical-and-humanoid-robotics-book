---
id: 1
title: Create AI Book Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: Hasnainbhopali
command: /sp.constitution
labels: [constitution, ai-book, rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
---

## Prompt

```
/sp.constitution

Project: AI-Native Book with Embedded RAG Chatbot

Purpose:
Create a spec-driven technical book using Spec-Kit Plus and Claude Code, published with Docusaurus on GitHub Pages, and enhanced with an embedded RAG chatbot grounded strictly in the book's content.

Core Principles:
- Specification-first development
- Accuracy and verifiability
- AI-native workflows
- Deterministic, non-hallucinatory RAG
- Reproducibility and clarity for developers

Standards:
- Authoring: Claude Code + Spec-Kit Plus
- Docs framework: Docusaurus (MDX)
- Deployment: GitHub Pages
- Single unified repository (specs, content, code)

Content Rules:
- Technical, concise, developer-focused
- No unsupported claims
- Use examples, diagrams, and code where useful

RAG Chatbot Rules:
- Answers must use only indexed book content
- Must support answering from user-selected text only
- If content is missing, respond with uncertainty
- Stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud

Technical Constraints:
- Free-tier compatible where specified
- Secure handling of secrets
- Fully documented build and deploy steps

Quality & Validation:
- Clean Docusaurus build
- Working GitHub Pages deployment
- Verified, grounded chatbot responses

Success Criteria:
- Published book with embedded chatbot
- Accurate, non-hallucinatory answers
- Reproducible repo and workflow
```

## Response snapshot

Created the project constitution file with all specified principles and constraints. The constitution includes six core principles: Specification-First Development, Accuracy and Verifiability, AI-Native Workflows, Deterministic Non-Hallucinatory RAG, Reproducibility and Clarity, and Free-Tier Compatibility. It also defines Technical Constraints, Development Workflow, and Governance sections based on the provided specifications.

## Outcome

- ✅ Impact: Created initial project constitution document with all required principles and constraints
- 🧪 Tests: Not applicable for constitution creation
- 📁 Files: Updated .specify/memory/constitution.md with the complete constitution
- 🔁 Next prompts: Proceed with feature specification and planning
- 🧠 Reflection: The constitution properly captures all the project's core principles and constraints as specified by the user

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin feature specification based on the established constitution