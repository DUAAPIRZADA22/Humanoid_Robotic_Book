# Claude Context for AI-Driven Book with RAG Chatbot Project

## Project Overview
Building an AI-driven book about "Physical AI & Humanoid Robotics" with a RAG-powered chatbot. The project uses a specification-driven development approach with MCP integration.

## Technology Stack

### Frontend (Module 1)
- **Framework**: Docusaurus 3.9 (Preset Classic)
- **Styling**: Tailwind CSS v4 with PostCSS
- **Deployment**: GitHub Pages with GitHub Actions CI/CD
- **UI Pattern**: Glassmorphism design with dark mode support
- **Animations**: Pure CSS for performance (no JavaScript animation libraries)

### Backend (Future Modules)
- **Framework**: FastAPI (planned)
- **Database**: Qdrant vector database (planned)
- **AI**: OpenAI embeddings and GPT models (planned)

## Key Architectural Decisions

### Frontend
1. **Static Site Generation**: Optimized for performance and SEO
2. **Mobile-First Design**: Progressive enhancement
3. **Component Architecture**: Modular React components following SRP
4. **Stateless Design**: No localStorage, all state in React components
5. **Animation Strategy**: Pure CSS animations to minimize bundle size

### Glassmorphism Implementation
- CSS backdrop-filter with Tailwind utilities
- Hardware acceleration for smooth performance
- Graceful fallback for Firefox users

### Dark Mode
- CSS custom properties with data-theme attribute
- Respects OS preference
- Smooth transitions without flash

## Project Structure

```
humanoid_robotic_book/
├── docs/                    # Docusaurus content
│   ├── intro.md
│   ├── module-1/            # Physical AI foundations
│   ├── module-2/            # Robotics fundamentals
│   ├── module-3/            # AI integration
│   └── module-4/            # Advanced topics
├── src/
│   ├── components/          # React components
│   │   ├── Hero/           # Landing page hero
│   │   ├── FeaturesGrid/   # Feature showcase
│   │   ├── CourseTimeline/ # Module overview
│   │   ├── FeatureButtons/ # Personalize/Translate buttons
│   │   └── Toast/          # Notification system
│   ├── css/               # Custom styles
│   ├── pages/             # Custom pages
│   │   └── index.js       # Landing page
│   └── theme/             # Swizzled theme
│       └── DocItem/       # Custom doc layout
├── static/               # Static assets
├── specs/                # Specification documents
│   └── docusaurus-frontend/
│       ├── spec.md       # Feature specification
│       ├── plan.md       # Implementation plan
│       ├── research.md   # Technical research
│       ├── data-model.md # Data model
│       ├── quickstart.md # Development guide
│       └── contracts/    # API contracts
├── .specify/             # SpecKit Plus configuration
├── .claude/              # Claude context (this file)
└── .github/              # CI/CD workflows
```

## Development Workflow

### MCP-Driven Process
1. **Specification** → `/sp.specify`
2. **Clarification** → `/sp.clarify`
3. **Planning** → `/sp.plan`
4. **Task Breakdown** → `/sp.tasks`
5. **Implementation** → `/sp.implement`

### Constitution Principles
- Specification-First Development
- Production-First Mindset
- Co-Learning with AI
- SOLID + DRY principles
- Performance > Speed
- Accessibility > Developer Convenience

## Current Module Status

### Module 1: Docusaurus Frontend
- **Status**: Planning complete, ready for implementation
- **Spec Version**: 1.0.0
- **Plan Version**: 1.0.0
- **Key Features**:
  - AI-native glassmorphism design
  - Responsive dark mode
  - Course structure for 4 modules
  - Bonus feature placeholders (personalization, translation)
  - GitHub Pages deployment

## Code Patterns

### Component Structure
```jsx
// src/components/Example/index.js
import React from 'react';
import './styles.css';

export default function Example({ children }) {
  return (
    <div className="example-component">
      {children}
    </div>
  );
}
```

### Tailwind CSS Usage
```css
/* Use utility classes */
.glass-card {
  @apply backdrop-blur-md bg-white/5 border border-white/10 rounded-xl p-6;
}

/* Custom theme extensions */
:root {
  --ifm-color-primary: #6366f1;
  --glass-white: rgba(255, 255, 255, 0.1);
}
```

### Error Boundaries
```jsx
class ErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false };
  }

  static getDerivedStateFromError(error) {
    return { hasError: true };
  }

  render() {
    if (this.state.hasError) {
      return <ErrorFallback />;
    }
    return this.props.children;
  }
}
```

## Testing Strategy

### Manual Testing (60-70%)
- Visual regression on multiple devices
- Accessibility testing with screen readers
- Performance testing with Lighthouse
- Cross-browser compatibility

### Automated Testing (20-30%)
- Lighthouse CI integration
- Accessibility testing with axe-core
- Bundle size analysis

### E2E Testing (5-10%)
- Critical user journeys
- Form interactions

## Performance Targets

### Core Web Vitals
- FCP: <1.5s
- LCP: <2.5s
- CLS: <0.1
- FID: <100ms

### Lighthouse Scores
- Performance: >90
- Accessibility: >95
- Best Practices: >90
- SEO: >90

## Deployment

### GitHub Pages Configuration
```javascript
// docusaurus.config.js
module.exports = {
  url: 'https://[username].github.io',
  baseUrl: '/humanoid_robotic_book/',
  organizationName: '[username]',
  projectName: 'humanoid_robotic_book',
  deploymentBranch: 'gh-pages',
};
```

### CI/CD Pipeline
- Trigger on push to main
- Build with Node.js 18
- Deploy via GitHub Actions
- Lighthouse CI validation
- Rollback on failures

## Documentation Standards

### Markdown Frontmatter
```markdown
---
title: "Chapter Title"
description: "Brief description"
module: 1
order: 1
difficulty: beginner
tags: ["tag1", "tag2"]
readingTimeMinutes: 10
lastUpdated: "2025-12-04"
published: true
---
```

### Code Comments
- Explain WHY, not WHAT
- Document complex logic
- Note performance considerations

## Accessibility Checklist

### WCAG AA Compliance
- [ ] Color contrast ratio >4.5:1
- [ ] Keyboard navigation support
- [ ] Screen reader compatibility
- [ ] Focus indicators visible
- [ ] Semantic HTML structure
- [ ] ARIA labels for interactive elements
- [ ] Skip navigation links
- [ ] Respects prefers-reduced-motion

## Security Considerations

### Frontend Security
- No sensitive data in client-side code
- Content Security Policy headers
- Subresource Integrity for external resources
- Validate all user inputs (future features)
- HTTPS only for external resources

## Debugging Tools

### Browser DevTools
- Performance tab for animation profiling
- Lighthouse audit for quality metrics
- Console for error tracking
- Network tab for resource loading

### VS Code Extensions
- ES7+ React/Redux/React-Native snippets
- Prettier - Code formatter
- ESLint
- Tailwind CSS IntelliSense

## Troubleshooting

### Common Issues
1. **Build fails**: Check Node.js version (18+), clear cache
2. **Styles not applying**: Verify Tailwind config, check CSS imports
3. **404 on deploy**: Ensure baseUrl matches repository name
4. **Performance issues**: Check bundle size, image optimization

## Context Updates
<!-- Manual additions between markers will be preserved -->

### Latest Additions (2025-12-04)
- Docusaurus 3.9 frontend plan completed
- Glassmorphism implementation strategy defined
- Tailwind CSS v4 integration approach finalized
- GitHub Pages deployment pipeline designed

### Next Steps
1. Begin implementation of Module 1
2. Create development environment
3. Implement core components
4. Set up CI/CD pipeline
5. Deploy to staging for review