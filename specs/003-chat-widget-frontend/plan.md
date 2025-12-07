# Implementation Plan: Chat Widget Frontend Integration

**Branch**: `003-chat-widget-frontend` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-chat-widget-frontend/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Building a production-ready React chat widget with glassmorphic design that connects to a Python FastAPI backend via Server-Sent Events (SSE). The widget will provide AI assistance for the Physical AI & Humanoid Robotics book, featuring text selection, local storage persistence, and a ChatGPT-like user experience.

Key technical decisions from research:
- Custom SSE implementation with useReducer to prevent infinite re-renders
- Glassmorphic design using CSS backdrop-filter and Tailwind utilities
- Text selection detection with smart popover positioning
- Encrypted localStorage for chat history persistence
- API key authentication via X-API-Key header

## Technical Context

**Language/Version**: TypeScript 5.x with React 18
**Primary Dependencies**: React, Docusaurus 3.x, Tailwind CSS, lucide-react, react-markdown, framer-motion, remark-gfm, crypto-js
**Storage**: Encrypted localStorage (AES-256)
**Testing**: Jest + React Testing Library (unit), Cypress (integration)
**Target Platform**: Web (modern browsers with backdrop-filter support)
**Project Type**: Frontend React component integrated into Docusaurus site
**Performance Goals**: Widget load <100ms, first message display <500ms, streaming latency <100ms, 60fps animations
**Constraints**: <2000 characters text selection, <100 messages stored, <50MB memory usage
**Scale/Scope**: Single-page widget, global mount across all doc pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### MCP Workflow Compliance ✅
- [x] Specification-First: Detailed spec with 14 functional requirements, 6 non-functional requirements, and measurable success criteria
- [x] Production-First: Error handling, retry mechanisms, logging, and monitoring included in design
- [x] Co-Learning: Human-AI clarification completed with 5 critical questions resolved

### SOLID Principles Validation ✅
- [x] SRP: Separate hooks for chat (useChatStream), text selection (useTextSelection), and storage
- [x] OCP: Configurable API endpoint, extensible message types, plugin-ready architecture
- [x] LSP: Message interface allows different message types (user, assistant, system)
- [x] ISP: Minimal interfaces, no unused dependencies in components
- [x] DIP: Depends on ApiConfig interface, not concrete implementation
- [x] DRY: Single state management pattern, reusable glassmorphism utilities

### Architecture Principles ✅
- [x] Separation of Concerns: UI layer, state management, API communication, storage layer
- [x] Statelessness: API requests stateless, widget state managed in React
- [x] Error Handling: Comprehensive error states with retry buttons and user feedback
- [x] Configuration: Environment-based via Docusaurus custom fields

### Quality Standards ✅
- [x] API Design: RESTful endpoints documented in OpenAPI spec, versioned /v1
- [x] TDD: Testing strategy defined (unit 60-70%, integration 20-30%, E2E 5-10%)
- [x] Testing Pyramid: Clear distribution and test types specified
- [x] Performance: Budgets defined (<100ms load, <500ms response, 60fps animations)
- [x] Documentation: Complete research, data model, and quickstart guides
- [x] Security: API key authentication, input sanitization, encrypted storage

### Development Practices ✅
- [x] Git Excellence: Feature branch `003-chat-widget-frontend` established
- [x] AI Collaboration: Used specialized skill for UI scaffolding, research agents for patterns
- [x] Decision Framework: Performance > Features, Security > Convenience prioritized
- [x] Success Metrics: 5 measurable outcomes defined with specific thresholds

## Project Structure

### Documentation (this feature)

```text
specs/003-chat-widget-frontend/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification with requirements
├── research.md          # Phase 0 output - Research findings and patterns
├── data-model.md        # Phase 1 output - Entity definitions and schemas
├── quickstart.md        # Phase 1 output - Implementation guide
├── contracts/           # Phase 1 output - API specifications
│   └── chat-api.yaml    # OpenAPI 3.0 spec for chat API
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── components/
│   └── ChatWidget/          # Main chat widget implementation
│       ├── index.tsx        # Main ChatWidget component
│       ├── components/      # UI components
│       │   ├── ChatInterface.tsx
│       │   ├── MessageBubble.tsx
│       │   ├── MessageInput.tsx
│       │   ├── SelectionPopover.tsx
│       │   └── ThinkingIndicator.tsx
│       ├── hooks/           # Custom React hooks
│       │   ├── useChatStream.ts
│       │   ├── useTextSelection.ts
│       │   └── useChatHistory.ts
│       ├── contexts/        # React contexts
│       │   ├── ChatContext.tsx
│       │   └── ApiContext.tsx
│       ├── utils/           # Utility functions
│       │   ├── api.ts       # API client
│       │   ├── storage.ts   # Encrypted localStorage
│       │   └── textProcessing.ts
│       ├── types/           # TypeScript definitions
│       │   ├── chat.ts      # Chat-related types
│       │   └── api.ts       # API-related types
│       └── styles/          # CSS modules
│           ├── ChatWidget.module.css
│           └── glassmorphism.css
├── theme/                  # Docusaurus theme customization
│   ├── Layout.tsx          # Swizzled layout to mount widget
│   └── Root.tsx            # Root component for global providers
├── css/
│   └── custom.css          # Global styles
└── pages/
    └── index.js            # Homepage (if modifications needed)
```

**Structure Decision**: React component-based architecture integrated into existing Docusaurus site. Widget is globally mounted via theme customization, with clear separation between UI components, business logic hooks, and utility functions.

## Complexity Tracking

> No constitution violations - all principles followed

## Architecture Decision Records

### ADR-001: State Management with useReducer
**Status**: Accepted
**Decision**: Use useReducer instead of multiple useState hooks for chat state
**Rationale**: Prevents infinite re-renders, provides atomic state updates, better performance
**Alternatives**: Zustand (overkill), Redux (too complex), useState (re-render issues)

### ADR-002: Custom SSE Implementation
**Status**: Accepted
**Decision**: Build custom Server-Sent Events implementation with AbortController
**Rationale**: Full control over reconnection logic, error handling, and performance
**Alternatives**: WebSocket libraries (bidirectional not needed), polling (inefficient)

### ADR-003: Glassmorphism via CSS
**Status**: Accepted
**Decision**: Implement glassmorphic design with CSS backdrop-filter
**Rationale**: Native browser performance, no runtime overhead, full customization
**Alternatives**: CSS-in-JS (abstraction), pre-built libraries (limited)

## Phase Summary

### Phase 0 Complete ✅
- Resolved all technical unknowns through research
- Documented best practices and anti-patterns
- Chose optimal patterns for performance and maintainability

### Phase 1 Complete ✅
- Defined complete data model with TypeScript interfaces
- Created OpenAPI 3.0 specification for backend contract
- Documented relationships between entities
- Prepared quickstart guide for implementation

## Next Steps

1. **Run `/sp.tasks`** to generate implementation tasks
2. **Begin implementation** following the project structure
3. **Follow the quickstart guide** for step-by-step setup
4. **Reference the research findings** for implementation patterns
5. **Use the data model** for type definitions and validation
