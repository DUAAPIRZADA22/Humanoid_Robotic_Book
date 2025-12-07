---
id: 001
title: Create Chatbot Backend Specification
stage: spec
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: 002-chatbot-backend
branch: 002-chatbot-backend
user: user
command: /sp.specify
labels: [specification, backend, rag, chatbot]
links:
  spec: specs/002-chatbot-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-chatbot-backend/spec.md
  - specs/002-chatbot-backend/checklists/requirements.md
tests:
  - Specification quality validation checklist
---

## Prompt

Context & Goals:
This is Module 3. We need a robust backend to serve the chatbot for the "Physical AI & Humanoid Robotics" book. This backend will ingest the book's content (Markdown files) and serve answers via a streaming API (Server-Sent Events) to our custom React frontend. We will deploy this to Hugging Face Spaces (Docker SDK).

Instructions for the Agent:

Use Sub-Agent & Skills:

Delegate the core implementation to the rag-specialist sub-agent if available, or directly utilize the rag-pipeline-builder skill capabilities.

Use mcp_context7_get-library-docs to verify latest best practices for fastapi, qdrant-client, and cohere SDK, if the skill templates need strict verification.

Customize for Deployment:

Update the Dockerfile to match Hugging Face Spaces requirements:

Expose port 7860 (not 8000).

Create a non-root user with ID 1000.

Set the working directory permissions correctly.

CMD should be:

uvicorn main:app --host 0.0.0.0 --port 7860


API Refinements:

Endpoint Protocol: The /chat endpoint MUST return Server-Sent Events (SSE).

Format:

data: {"content": "text chunk", "done": false}


Metadata Event (first event):

data: {"metadata": {...}, "done": false}


Done Event:

data: {"done": true}


Ingestion:

Add a POST /ingest endpoint that triggers the ingest_documents.py logic.

Ensure it can read files from the book_content directory (copied into the image or mounted).

CORS:

Configure CORSMiddleware to explicitly allow:

https://github.com/DUAAPIRZADA22

http://localhost:3000

LangChain-Free Implementation:

Strictly follow the "LangChain-free" architecture.

Use the pure Python chunking_example.py logic for splitting text, with the added improvement:

min_chunk_size = 50 (to filter out empty headers).

Implement deduplication during retrieval (fetch 2× top_k, then filter unique text chunks).

Configuration:

Generate a .env.example that includes:

COHERE_API_KEY

QDRANT_URL

QDRANT_API_KEY

Deliverables:
A complete backend/ directory containing:

main.py (FastAPI app with lifespan events)

rag/ package (chunking, embedding, vector logic)

Dockerfile (HF Spaces optimized, Port 7860)

requirements.txt (pinned versions)

scripts/ (ingestion utilities)

Constraint Checklist & Confidence Score:

Use rag-pipeline-builder skill? Yes.

Hugging Face Spaces (Port 7860)? Yes.

SSE Streaming (Custom Format)? Yes.

No LangChain? Yes.

Deduplication & Min Chunk Size? Yes.

Confidence Score: 5/5

Key Libraries:

fastapi

uvicorn

cohere

qdrant-client

tiktoken

(No langchain)

## Response snapshot

Successfully created feature branch 002-chatbot-backend and generated a comprehensive specification document. The specification includes:

- 3 prioritized user stories (Document Ingestion P1, Real-time Chat Responses P1, Content Search and Retrieval P2)
- 10 functional requirements covering ingestion, streaming API, deduplication, and deployment
- 5 measurable success criteria with specific metrics
- Complete edge case analysis
- Quality checklist created and validated

All validation criteria passed - the specification is ready for the next phase.

## Outcome

- ✅ Impact: Created complete, validated specification for chatbot backend feature
- 🧪 Tests: Specification quality checklist fully validated (16/16 criteria passed)
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for /sp.plan or /sp.clarify if needed
- 🧠 Reflection: Specification successfully captures all requirements without implementation details

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All quality checks PASSED
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): None required - specification is complete and validated