# AI-Driven Book with RAG Chatbot Constitution
<!-- Version: 1.0.0 - MCP-Ready Production Constitution -->

## Core Development Philosophy (MCP Integrated)

### I. Specification-First Development (Non-Negotiable)
Every feature starts with a detailed specification defining intent, constraints, and acceptance criteria. Vague requirements cause exponential rework with AI literalness.

**MCP Application**: Follow /constitution → /specify → /clarify → /plan → /tasks → /implement workflow rigorously.

**Rationale**: AI literalness requires precision; specifications are living contracts, not afterthoughts.

**Checkpoint**: MCP "Spec Validation" must pass before coding begins.

### II. Production-First Mindset
Build for production from day one. All components must be deployable, observable, and maintainable from inception.

**MCP Application**: Include deployment, monitoring, and error handling in every specification.

**Rationale**: Technical debt compounds exponentially; prevention is 10x cheaper than remediation.

**Checkpoint**: MCP "Deployable Component" status required before merge approval.

### III. Co-Learning with AI
Human-AI collaboration through iterative dialogue, not command-response pattern. AI suggests patterns based on specifications; human validates and refines.

**MCP Application**: Use /clarify to question AI suggestions, provide constraints, update specifications iteratively.

**Rationale**: Combining AI pattern recognition with human judgment yields optimal results.

## Code Quality Standards (SOLID + DRY, MCP-Aligned)

### IV. Single Responsibility Principle (SRP)
Each module, class, and function must have exactly one reason to change.

**Book Project**: Content generation is separate from rendering; RAG retrieval is separate from response generation.

**Chatbot**: Embedding logic is separate from vector storage; API routing is separate from business logic.

**Checkpoint**: /tasks/validate-module confirms SRP compliance for every component.

### V. Open/Closed Principle (OCP)
Software entities must be open for extension but closed for modification.

**Book Project**: Chapter templates extendable without altering core generation engine.

**Chatbot**: RAG pipeline supports multiple vector stores without code rewrite.

**Checkpoint**: /specify/feature-extension approval required for any core modifications.

### VI. Liskov Substitution Principle (LSP)
Subtypes must be replaceable by their base types without altering correctness.

**Book Project**: Tutorial/reference/conceptual chapters interchangeable in rendering pipeline.

**Chatbot**: Semantic/keyword/hybrid retrieval strategies interchangeable.

**Checkpoint**: /tasks/test-subtype confirms seamless substitution works.

### VII. Interface Segregation Principle (ISP)
Clients must not depend on interfaces they don't use.

**Book Project**: Only technical chapters implement generateCodeExamples interface.

**Chatbot**: ChatService and EmbeddingService interfaces are separate and focused.

**Checkpoint**: /tasks/interface-audit passed before interface approval.

### VIII. Dependency Inversion Principle (DIP)
High-level modules must not depend on low-level modules; both depend on abstractions.

**Book Project**: Depend on ContentGenerator interface, not DocusaurusGenerator concretion.

**Chatbot**: Depend on VectorStore interface, not Qdrant implementation directly.

**Checkpoint**: /specify/abstract-dependency verified during architecture review.

### IX. Don't Repeat Yourself (DRY)
Every piece of knowledge must have a single, unambiguous representation.

**Book Project**: Chapter metadata centralized in configuration, not scattered across files.

**Chatbot**: Chunking, embeddings, and error handling defined once, reused throughout.

**Checkpoint**: /tasks/dry-check automated validation passes.

## Architecture Principles

### X. Separation of Concerns
Clear boundaries between layers with minimal coupling.

**Frontend (Docusaurus)**:
- Content (Markdown chapters)
- Components (React - ChatWidget)
- Configuration (docusaurus.config.js)

**Backend (FastAPI)**:
- API Layer (routes and endpoints)
- Business Logic (RAG pipeline)
- Data Layer (Qdrant integration)
- Infrastructure (config, logging, monitoring)

**MCP Integration**: /plan/architecture-review required for approval of any cross-cutting changes.

### XI. Statelessness
Stateless design for scalability and reliability.

**Frontend**: Chat state managed in React components only; no localStorage persistence.

**Backend**: RESTful APIs are stateless; all context explicit in requests.

**Checkpoint**: /tasks/state-validation confirms no hidden state dependencies.

## Error Handling and Configuration

### XII. Defensive Error Handling
Robust error handling with proper logging and user-friendly messages.

```python
try:
    result = await external_service.call()
except SpecificException as e:
    logger.error(f"Context: {e}")
    raise HTTPException(status_code=500, detail="Friendly message")
```

**Rules**: Never expose internals, log all errors, implement retries for transient failures.

**Checkpoint**: /tasks/error-handling verified for all external integrations.

### XIII. Configuration Management
Environment-based configuration with validation.

```python
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    openai_api_key: str
    qdrant_url: str
    qdrant_api_key: str

    class Config:
        env_file = ".env"
```

**Rules**: All secrets in environment variables, separate configs for dev/staging/prod, fail-fast validation.

**Checkpoint**: /tasks/config-validation confirms all settings properly managed.

## API and Testing Standards

### XIV. RESTful API Design
Versioned, predictable, and well-documented APIs.

**Standards**:
- Versioned endpoints (/v1)
- RESTful conventions
- Comprehensive schema documentation
- Consistent error responses

**Checkpoint**: /tasks/api-review validates all endpoint designs.

### XV. Test-Driven Development (TDD)
Tests written before implementation, no exceptions.

**Rule**: Red-Green-Refactor cycle strictly enforced for all features.

**Checkpoint**: /tasks/tdd-approved confirms TDD process followed.

### XVI. Testing Pyramid
Balanced testing approach with appropriate coverage levels.

**Distribution**: Unit (60-70%) → Integration (20-30%) → E2E (5-10%)

**Book Project**: Markdown parsing, Docusaurus pipeline, deployed navigation.

**Chatbot**: Chunking, embedding, RAG flow integration tests.

**Checkpoint**: /tasks/test-pyramid validates coverage distribution.

## Performance and Documentation

### XVII. Performance Budgets
Explicit performance requirements with monitoring.

**Book**: <2s load time on 3G connection, lazy load images, minimal JavaScript, SSG optimization.

**Chatbot**: <3s first response, <100ms token latency, response caching, parallel embedding generation.

**Cost**: OpenAI embeddings batched, Qdrant free tier optimized.

**Checkpoint**: /tasks/performance-metrics continuously monitored.

### XVIII. Comprehensive Documentation
Complete documentation for production readiness.

**Requirements**:
- README: Features, architecture, setup, deployment, environment variables, API examples, troubleshooting
- Code comments: Explain WHY, not WHAT
- Inline docs: Docstrings with metadata and explanations

**Checkpoint**: /tasks/docs-approved confirms documentation completeness.

## Security and Development Practices

### XIX. Security-First Development
Security considerations integrated throughout development.

**API Security**: No committed secrets, CORS configuration, rate limiting, input validation.

**Dependencies**: Pinned versions, minimal required packages, regular pip-audit scans.

**Error Exposure**: Generic production messages, detailed development logs, alerting.

**Checkpoint**: /tasks/security-review passed before deployment.

### XX. Git Excellence
Disciplined version control practices.

**Standards**:
- Commit messages: type(scope): subject format
- Branching: main/develop/feature branches
- PR Checklist: tests pass, docs updated, no secrets, error handling, performance, accessibility

**Checkpoint**: /tasks/git-compliance validates all changes.

### XXI. AI Collaboration Guidelines
Effective human-AI partnership practices.

**Subagents**: Specialized tasks with separate context, reusable knowledge patterns.

**Skills**: Repeatable patterns, template-driven work with consistency.

**Prompt Quality**: Good prompts are MCP-reviewed, bad prompts are identified and avoided.

**Checkpoint**: /tasks/ai-collab-approval for all AI-assisted work.

## Decision Framework and Success Metrics

### XXII. Decision-Making Hierarchy
Clear priority ordering for technical decisions.

**Priority Order**:
1. Correctness > Speed
2. Simplicity > Cleverness
3. User Experience > Developer Convenience
4. Maintainability > Lines of Code
5. Security > Features

**Red Flags**: Hardcoding secrets, skipping tests, "works on my machine", "refactor later" promises.

**Checkpoint**: /tasks/decision-review validates architectural choices.

### XXIII. Quantifiable Success Metrics
Measurable criteria for project success.

**Book Metrics**:
- ≥10 chapters with 2-4k words each
- Clear content hierarchy with code examples
- Mobile-responsive design
- Lighthouse score 90+

**Chatbot Metrics**:
- Accurate contextual answers
- Text selection for Q&A
- Streaming responses <100ms latency
- Mobile-friendly interface
- Relevance score ≥8/10

**Deployment Metrics**:
- GitHub Pages automated deployment
- Backend running on free tier
- Environment variables documented
- Health endpoint with monitoring
- CORS and SSL configured

**Checkpoint**: /tasks/success-metrics tracked throughout development.

## Governance

### Amendment Process
Constitution amendments require:
1. Formal documentation of proposed changes
2. Team review and approval
3. Migration plan for existing work
4. Version increment following semantic versioning
5. Updated consistency across all dependent artifacts

### Compliance Review
All development activities must verify compliance:
- Plan templates must include Constitution Check gates
- Task categorization must reflect principle requirements
- Code reviews must validate principle adherence
- MCP checkpoints must validate before merge approval

### Version Policy
- **MAJOR**: Backward incompatible governance changes or principle redefinitions
- **MINOR**: New principles or substantial guidance expansions
- **PATCH**: Clarifications, wording improvements, non-semantic refinements

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04