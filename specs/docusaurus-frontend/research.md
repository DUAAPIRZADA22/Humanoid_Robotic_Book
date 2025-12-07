# Phase 0: Research Findings

**Project**: Physical AI & Humanoid Robotics Book - Docusaurus Frontend
**Date**: 2025-12-04

## Glassmorphism Implementation

### Decision: CSS backdrop-filter with Tailwind utilities

**Rationale**:
- Native browser support in Chrome, Edge, Safari (behind flag in Firefox)
- Zero runtime cost - pure CSS
- Seamless integration with Tailwind utility classes
- Hardware acceleration for smooth performance

**Implementation Approach**:
```css
.glass {
  backdrop-filter: blur(12px);
  background: rgba(255, 255, 255, 0.1);
  border: 1px solid rgba(255, 255, 255, 0.2);
}
```

**Alternatives Considered**:
- JavaScript libraries (e.g., react-glassmorphism): Rejected due to bundle size impact
- SVG filters: Rejected due to complexity and performance overhead
- Pre-rendered images: Rejected due to flexibility limitations

**Support Note**: Firefox requires `layout.css.backdrop-filter.enabled` flag. Will provide graceful fallback.

## Animation Strategy

### Decision: Pure CSS animations and transitions

**Rationale**:
- 60fps performance with GPU acceleration
- Zero JavaScript overhead
- Respects `prefers-reduced-motion` automatically
- No additional dependencies

**Performance Characteristics**:
```css
@keyframes fadeInUp {
  from {
    opacity: 0;
    transform: translateY(30px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}
```

**Alternatives Considered**:
- Framer Motion: Rejected - adds 44KB to bundle, overkill for simple animations
- GSAP (GreenSock): Rejected - commercial license required for production
- React Spring: Rejected - adds complexity, CSS is sufficient

## Tailwind CSS v4 Integration

### Decision: PostCSS plugin with custom theme extension

**Latest Version Research**: Tailwind CSS v4.0.0-alpha.25
- New CSS engine for better performance
- Improved content detection
- Enhanced JIT mode
- Better TypeScript support

**Configuration Strategy**:
```javascript
// tailwind.config.js
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './docs/**/*.{md,mdx}',
    './src/theme/**/*.js',
  ],
  darkMode: 'class',
  theme: {
    extend: {
      colors: {
        glass: {
          white: 'rgba(255, 255, 255, 0.1)',
          border: 'rgba(255, 255, 255, 0.2)',
        }
      },
      backdropBlur: {
        xs: '2px',
      }
    }
  }
}
```

**Alternatives Considered**:
- Tailwind v3.4: Rejected - missing v4 features and performance improvements
- Styled Components: Rejected - violates "must use Tailwind CSS" constraint
- Emotion: Rejected - same as above

## GitHub Pages Optimization

### Decision: Asset base URL configuration with absolute paths

**Research Findings**:
- GitHub Pages serves from repository subdirectories by default
- Base URL must match repository name
- All asset references need to be absolute

**Configuration Pattern**:
```javascript
// docusaurus.config.js
module.exports = {
  baseUrl: '/humanoid_robotic_book/',
  url: 'https://username.github.io',
  organizationName: 'username',
  projectName: 'humanoid_robotic_book',
  deploymentBranch: 'gh-pages',
}
```

**Best Practices**:
- Use `withBaseUrl` helper for all internal links
- Configure static asset paths correctly
- Test build locally with `--base-url` flag

**Alternatives Considered**:
- Relative paths: Rejected - causes navigation issues with GitHub Pages subdirectory
- Rewrite rules: Rejected - GitHub Pages doesn't support server-side rewrites
- SPA fallback: Rejected - unnecessary for static site

## Dark Mode Implementation

### Decision: CSS custom properties with data-theme attribute

**Research Findings**:
- Docusaurus natively supports dark mode toggling
- Uses `data-theme="dark"` on HTML element
- Automatic OS preference detection
- Smooth transitions without flash

**Implementation Pattern**:
```css
:root {
  --background: #ffffff;
  --foreground: #000000;
}

[data-theme='dark'] {
  --background: #0f172a;
  --foreground: #f1f5f9;
}

body {
  background-color: var(--background);
  color: var(--foreground);
  transition: background-color 0.3s ease, color 0.3s ease;
}
```

**Alternatives Considered**:
- Class-based theming: Rejected - requires state management
- Separate CSS files: Rejected - maintenance overhead
- Media queries only: Rejected - no user control

## Typography Strategy

### Decision: Inter font with system font fallbacks

**Research**:
- Inter is optimized for UI and readability
- Variable font support for performance
- Self-hosted via Google Fonts
- System font fallbacks for loading state

**Implementation**:
```css
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&display=swap');

:root {
  --font-inter: 'Inter', system-ui, -apple-system, sans-serif;
}
```

**Performance Considerations**:
- `font-display: swap` for better loading
- Preload critical font weights
- Subset to Latin characters initially

## Accessibility Implementation

### WCAG AA Compliance Strategy

**Color Contrast Research**:
- All text must meet 4.5:1 ratio
- Large text (18px+) requires 3:1 ratio
- Interactive elements need enhanced contrast

**Implementation Approach**:
```css
/* Accessible focus indicators */
:focus-visible {
  outline: 2px solid #6366f1;
  outline-offset: 2px;
}

/* Screen reader only content */
.sr-only {
  position: absolute;
  width: 1px;
  height: 1px;
  padding: 0;
  margin: -1px;
  overflow: hidden;
  clip: rect(0, 0, 0, 0);
  white-space: nowrap;
}
```

**Tools for Validation**:
- axe-core browser extension
- Chrome DevTools Lighthouse audit
- WAVE Web Accessibility Evaluation Tool
- Manual keyboard navigation testing

## Performance Optimization

### Bundle Size Strategy

**Research Findings**:
- Docusaurus default bundle: ~200KB
- Tailwind CSS purge: Reduces to ~10KB
- Code splitting: Route-based chunks
- Image optimization: WebP with fallbacks

**Implementation Tactics**:
1. Purge unused Tailwind utilities
2. Lazy load non-critical images
3. Minify CSS/JS production builds
4. Gzip compression (GitHub Pages default)

### Loading Performance

**Critical Rendering Path**:
```javascript
// Preload critical fonts
<link rel="preload" href="/fonts/inter-regular.woff2" as="font" type="font/woff2" crossorigin>

// DNS prefetch for external domains
<link rel="dns-prefetch" href="//fonts.googleapis.com">
```

**Metrics Targets**:
- First Contentful Paint: <1.5s
- Time to Interactive: <3s
- Cumulative Layout Shift: <0.1

## Component Architecture

### React Component Strategy

**Decision**: Functional components with hooks
- Simpler than class components
- Better TypeScript support
- Smaller bundle size

**State Management**:
- Local state with useState/useReducer
- No global state management needed
- Props drilling acceptable for this scope

**Error Handling**:
```javascript
// Error boundary wrapper
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
      return <h1>Something went wrong.</h1>;
    }
    return this.props.children;
  }
}
```

## Documentation from Docusaurus

### Latest Configuration Best Practices

**From `/facebook/docusaurus` docs**:

1. **Base URL Configuration**: Must match GitHub Pages subdirectory
2. **Organization/Project**: Required for automatic deployment
3. **Deployment Branch**: Default is `gh-pages`
4. **Custom CSS**: Imported via theme configuration
5. **Presets**: Classic preset includes docs, blog, and theme

### Swizzling Best Practices

**DocItem Component**:
- Use `npm run swizzle @docusaurus/theme-classic DocItem --wrap`
- Creates wrapper in `src/theme/DocItem/index.js`
- Preserve original functionality with `<OriginalDocItem />`

**Navigation Customization**:
- Swizzle Navbar for custom items
- Use themeConfig for simple modifications

---

**Research Summary**: All technical unknowns resolved with clear implementation paths. Proceeding to Phase 1 design and contracts.