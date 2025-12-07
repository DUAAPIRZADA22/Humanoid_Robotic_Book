# Quickstart Guide

**Project**: Physical AI & Humanoid Robotics Book
**Module**: 1 - Frontend Foundation
**Date**: 2025-12-04

## Prerequisites

### Required Software
- **Node.js**: 18.x or later (LTS recommended)
- **npm**: 9.x or later
- **Git**: Latest version
- **VS Code** (recommended) with extensions:
  - ES7+ React/Redux/React-Native snippets
  - Prettier - Code formatter
  - ESLint
  - Tailwind CSS IntelliSense

### System Requirements
- RAM: 4GB minimum, 8GB recommended
- Storage: 1GB free space
- Network: Stable internet connection for dependencies

## Installation Steps

### 1. Clone Repository
```bash
git clone https://github.com/[username]/humanoid_robotic_book.git
cd humanoid_robotic_book
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Environment Setup
Create a `.env.local` file for local development:
```bash
# .env.local
GOOGLE_ANALYTICS_ID=your-ga-id
NODE_ENV=development
```

### 4. Start Development Server
```bash
npm start
```
Navigate to `http://localhost:3000`

## Development Workflow

### Daily Development
1. **Start**: `npm start` for development server
2. **Code**: Edit files in `src/` and `docs/`
3. **Format**: `npm run format` (Prettier)
4. **Lint**: `npm run lint` (ESLint)
5. **Test**: `npm run test` (if tests exist)
6. **Build Check**: `npm run build` to verify

### Adding New Content

#### Create New Chapter
1. Create markdown file in `docs/`:
   ```markdown
   ---
   title: "Chapter Title"
   description: "Brief description"
   module: 1
   order: 1
   difficulty: beginner
   tags: ["tag1", "tag2"]
   ---

   # Chapter Title

   Content here...
   ```

2. Add to `sidebars.js`:
   ```javascript
   module.exports = {
     tutorialSidebar: [
       {
         type: 'doc',
         id: 'chapter-slug',
         label: 'Chapter Title',
       },
     ],
   };
   ```

#### Create New Component
1. Create component in `src/components/`:
   ```jsx
   // src/components/MyComponent/index.js
   import React from 'react';
   import './styles.css';

   export default function MyComponent() {
     return <div>My Component</div>;
   }
   ```

2. Create styles:
   ```css
   /* src/components/MyComponent/styles.css */
   .my-component {
     @apply p-4 bg-white rounded-lg shadow-md;
   }
   ```

3. Use in page:
   ```jsx
   import MyComponent from '@site/src/components/MyComponent';

   <MyComponent />
   ```

## Common Tasks

### Adding Custom Styles
1. Edit `src/css/custom.css`:
   ```css
   /* Custom theme variables */
   :root {
     --ifm-color-primary: #6366f1;
     --ifm-color-info: #0ea5e9;
   }

   /* Custom animations */
   @keyframes fadeIn {
     from { opacity: 0; }
     to { opacity: 1; }
   }
   ```

### Configuring Tailwind
Edit `tailwind.config.js`:
```javascript
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './docs/**/*.{md,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        primary: '#6366f1',
        secondary: '#8b5cf6',
      },
    },
  },
};
```

### Adding Images
1. Place image in `static/img/`:
   ```
   static/img/
   ├── logo.png
   ├── hero-bg.jpg
   └── features/
   ```

2. Reference in Markdown:
   ```markdown
   ![Alt text](/img/logo.png)
   ```

3. Reference in React:
   ```jsx
   import Logo from '@site/static/img/logo.png';
   <img src={Logo} alt="Logo" />
   ```

## Build and Deployment

### Local Build
```bash
npm run build
```
Build output in `build/` directory.

### Preview Production Build
```bash
npm run serve
```
Preview at `http://localhost:3000`

### Deploy to GitHub Pages
```bash
git add .
git commit -m "feat: update content"
git push origin main
```

GitHub Actions will automatically deploy to GitHub Pages.

## Project Structure

```
humanoid_robotic_book/
├── docs/                   # Content files
│   ├── intro.md           # Introduction page
│   ├── module-1/          # Module 1 chapters
│   └── assets/            # Content assets
├── src/                   # Custom source code
│   ├── components/        # React components
│   │   ├── Hero/          # Hero section
│   │   ├── FeaturesGrid/  # Features grid
│   │   └── Toast/         # Toast notifications
│   ├── css/               # Custom styles
│   ├── pages/             # Custom pages
│   │   └── index.js       # Landing page
│   └── theme/             # Swizzled theme
│       └── DocItem/       # Custom doc layout
├── static/                # Static assets
│   ├── img/               # Images
│   └── assets/            # Other assets
├── .github/               # GitHub Actions
│   └── workflows/
│       └── deploy.yml     # Deployment workflow
├── docusaurus.config.js   # Docusaurus config
├── tailwind.config.js     # Tailwind config
├── sidebars.js            # Navigation sidebar
└── package.json           # Dependencies and scripts
```

## Available Scripts

```json
{
  "scripts": {
    "start": "docusaurus start",
    "build": "docusaurus build",
    "serve": "docusaurus serve",
    "deploy": "docusaurus deploy",
    "swizzle": "docusaurus swizzle",
    "clear": "docusaurus clear",
    "write-translations": "docusaurus write-translations",
    "write-heading-ids": "docusaurus write-heading-ids",
    "format": "prettier --write \"**/*.{js,jsx,ts,tsx,json,md,css}\"",
    "lint": "eslint \"src/**/*.{js,jsx,ts,tsx}\"",
    "lint:fix": "eslint \"src/**/*.{js,jsx,ts,tsx}\" --fix",
    "test": "npm run lint",
    "test:build": "npm run build && npm run serve",
    "validate-config": "node scripts/validate-config.js",
    "check-links": "remark-cli --no-stdout --quiet",
    "clean": "npm run clear && rm -rf node_modules package-lock.json"
  }
}
```

## Configuration Files

### docusaurus.config.js
Main configuration file for:
- Site metadata
- Navigation structure
- Theme customization
- Plugin configuration

### tailwind.config.js
Tailwind CSS configuration for:
- Theme extensions
- Custom colors
- Spacing system
- Breakpoints

### sidebars.js
Navigation structure for:
- Module organization
- Chapter ordering
- Auto-generated TOC

## Custom Components

### Hero Section
Location: `src/components/Hero/index.js`
```jsx
<Hero
  title="Physical AI & Humanoid Robotics"
  subtitle="Master embodied intelligence"
  ctaText="Start Reading"
  ctaHref="/docs/intro"
/>
```

### Features Grid
Location: `src/components/FeaturesGrid/index.js`
```jsx
<FeaturesGrid
  features={[
    {
      id: 'feature-1',
      title: 'Interactive Content',
      description: 'Learn by doing',
      icon: '🎯',
    }
  ]}
  columns={3}
/>
```

### Toast Notifications
Location: `src/components/Toast/index.js`
```javascript
import { useToast } from '@site/src/components/Toast/context';

function MyComponent() {
  const { addToast } = useToast();

  return (
    <button
      onClick={() => addToast({
        message: 'Coming soon!',
        type: 'info',
        autoDismiss: 3000
      })}
    >
      Click me
    </button>
  );
}
```

## Styling Guidelines

### Tailwind Usage
- Use utility classes for styling
- Avoid inline styles
- Component-specific styles in CSS modules
- Follow mobile-first approach

### CSS Architecture
```css
/* Base styles in custom.css */
:root {
  --ifm-color-primary: #6366f1;
}

/* Component styles */
.hero {
  @apply bg-gradient-to-br from-blue-900 to-purple-900;
}

/* Responsive design */
@media (min-width: 768px) {
  .hero {
    @apply py-24;
  }
}
```

## Troubleshooting

### Common Issues

#### Build Errors
```bash
# Clear cache
npm run clean

# Reinstall dependencies
rm -rf node_modules package-lock.json
npm install

# Check Node version
node --version  # Should be 18.x or later
```

#### Styling Issues
```bash
# Check Tailwind config
npx tailwindcss --help

# Validate CSS
npm run lint
```

#### Content Not Showing
1. Check file paths in `sidebars.js`
2. Verify frontmatter in markdown files
3. Check for syntax errors

### Getting Help

1. **Check Console**: Browser DevTools for errors
2. **Build Output**: `npm run build` for detailed errors
3. **Documentation**: [Docusaurus Docs](https://docusaurus.io/docs)
4. **Community**: GitHub Issues and Discussions

## Best Practices

### Code Organization
- Keep components small and focused
- Use meaningful file and component names
- Group related functionality
- Document complex logic

### Performance
- Lazy load images
- Optimize bundle size
- Use code splitting
- Minimize re-renders

### Accessibility
- Use semantic HTML
- Add ARIA labels
- Ensure keyboard navigation
- Test with screen readers

### SEO
- Use proper heading hierarchy
- Add meta descriptions
- Create sitemap (automatic)
- Use structured data

## Next Steps

1. **Review**: Familiarize yourself with the codebase
2. **Customize**: Update colors, fonts, and branding
3. **Add Content**: Create initial chapters
4. **Test**: Verify all functionality
5. **Deploy**: Push to GitHub for automatic deployment

## Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [Tailwind CSS Documentation](https://tailwindcss.com/docs)
- [React Documentation](https://reactjs.org/docs)
- [GitHub Pages Documentation](https://docs.github.com/en/pages)
- [Web.dev Accessibility Guide](https://web.dev/accessibility/)