# Data Model

**Project**: Physical AI & Humanoid Robotics Book
**Module**: 1 - Frontend Foundation
**Version**: 1.0.0
**Date**: 2025-12-04

## Entity Overview

This is a static site with minimal runtime data. Most content is authored in Markdown and processed at build time.

## Core Entities

### Chapter
Represents a single chapter in the course.

```typescript
interface Chapter {
  /** Unique identifier for the chapter */
  id: string;

  /** Chapter title displayed in navigation */
  title: string;

  /** Brief description for cards and meta */
  description: string;

  /** Module this chapter belongs to (1-4) */
  module: 1 | 2 | 3 | 4;

  /** Order within the module */
  order: number;

  /** URL slug for the chapter */
  slug: string;

  /** Full content in Markdown */
  content: string;

  /** Last modification timestamp */
  lastUpdated: Date;

  /** Estimated reading time in minutes */
  readingTimeMinutes: number;

  /** Whether chapter is published */
  published: boolean;

  /** Chapter tags for filtering */
  tags: string[];

  /** Chapter difficulty level */
  difficulty: 'beginner' | 'intermediate' | 'advanced';

  /** Prerequisite chapters */
  prerequisites: string[];
}
```

### Module
Represents a course module containing related chapters.

```typescript
interface Module {
  /** Module number (1-4) */
  id: 1 | 2 | 3 | 4;

  /** Module title */
  title: string;

  /** Module description */
  description: string;

  /** List of chapters in this module */
  chapters: Chapter[];

  /** Display order */
  order: number;

  /** Module icon or emoji */
  icon?: string;

  /** Module color theme */
  color?: string;

  /** Estimated completion time in hours */
  estimatedHours: number;

  /** Module status */
  status: 'draft' | 'in-progress' | 'completed';
}
```

### Feature
Represents a feature showcase on the landing page.

```typescript
interface Feature {
  /** Unique identifier */
  id: string;

  /** Feature title */
  title: string;

  /** Feature description */
  description: string;

  /** Icon name or emoji */
  icon: string;

  /** Feature image (optional) */
  image?: string;

  /** Whether feature is coming soon */
  comingSoon?: boolean;

  /** Call to action text */
  ctaText?: string;

  /** Call to action link */
  ctaLink?: string;

  /** Feature category */
  category: 'content' | 'interaction' | 'accessibility' | 'performance';
}
```

### SiteConfiguration
Global site configuration.

```typescript
interface SiteConfiguration {
  /** Site title */
  title: string;

  /** Site tagline */
  tagline: string;

  /** Full site URL */
  url: string;

  /** Base URL for subdirectory deployment */
  baseUrl: string;

  /** GitHub organization or username */
  organizationName: string;

  /** Repository name */
  projectName: string;

  /** Deployment branch */
  deploymentBranch: string;

  /** Custom domain (optional) */
  customDomain?: string;

  /** Site metadata */
  metadata: {
    /** Site description */
    description: string;

    /** Keywords for SEO */
    keywords: string[];

    /** Author information */
    author: {
      name: string;
      email?: string;
      url?: string;
    };
  };

  /** Social media links */
  social: {
    github?: string;
    twitter?: string;
    linkedin?: string;
    email?: string;
  };

  /** Analytics configuration */
  analytics?: {
    googleAnalyticsId?: string;
    plausibleDomain?: string;
  };
}
```

### ToastNotification
Configuration for toast notifications.

```typescript
interface ToastNotification {
  /** Unique identifier */
  id: string;

  /** Notification message */
  message: string;

  /** Notification type */
  type: 'info' | 'success' | 'warning' | 'error';

  /** Auto-dismiss timeout in milliseconds */
  autoDismiss?: number;

  /** Whether to show close button */
  showCloseButton?: boolean;

  /** Position on screen */
  position: 'top-right' | 'top-left' | 'bottom-right' | 'bottom-left';
}
```

## Relationships

```
SiteConfiguration (1)
├── Module (4)
│   └── Chapter (1..N)
├── Feature (1..N)
└── ToastNotification (0..N, runtime only)
```

## Validation Rules

### Chapter
- `id`: Required, unique, alphanumeric with hyphens
- `title`: Required, max 100 characters
- `description`: Required, max 500 characters
- `module`: Required, must be 1-4
- `order`: Required, positive integer
- `slug`: Required, matches pattern `/^[a-z0-9-]+$/`
- `readingTimeMinutes`: Optional, positive integer
- `difficulty`: Optional, must be valid enum value

### Module
- `id`: Required, unique, must be 1-4
- `title`: Required, max 100 characters
- `description`: Required, max 500 characters
- `order`: Required, positive integer
- `estimatedHours`: Optional, positive number
- `status`: Optional, must be valid enum value

### Feature
- `id`: Required, unique, alphanumeric with hyphens
- `title`: Required, max 100 characters
- `description`: Required, max 500 characters
- `icon`: Required, max 50 characters
- `category`: Required, must be valid enum value

### SiteConfiguration
- `title`: Required, max 100 characters
- `url`: Required, valid URL
- `baseUrl`: Required, must start with `/`
- `organizationName`: Required, non-empty string
- `projectName`: Required, valid GitHub repository name

## State Transitions

### Module
```
draft → in-progress → completed
```

### ToastNotification (Runtime)
```
created → showing → dismissed
```

## Default Values

### Chapter
```typescript
const defaultChapter: Partial<Chapter> = {
  published: true,
  readingTimeMinutes: 0,
  tags: [],
  difficulty: 'beginner',
  prerequisites: []
};
```

### Module
```typescript
const defaultModule: Partial<Module> = {
  chapters: [],
  status: 'draft',
  estimatedHours: 0
};
```

### Feature
```typescript
const defaultFeature: Partial<Feature> = {
  comingSoon: false,
  category: 'content'
};
```

### ToastNotification
```typescript
const defaultToast: Partial<ToastNotification> = {
  type: 'info',
  autoDismiss: 3000,
  showCloseButton: true,
  position: 'top-right'
};
```

## Data Sources

### Static Data (Build Time)
- Markdown files in `docs/` directory
- Frontmatter in each MDX file
- `docusaurus.config.js` configuration
- `sidebars.js` navigation structure

### Runtime Data
- Theme preference (localStorage)
- Toast notifications (component state)
- Scroll position (browser API)

## Content Structure Example

```markdown
---
title: "Introduction to Physical AI"
description: "Learn the fundamentals of physical AI and embodied intelligence"
module: 1
order: 1
difficulty: beginner
tags: ["ai", "robotics", "introduction"]
readingTimeMinutes: 10
lastUpdated: "2025-12-04"
published: true
---

# Introduction to Physical AI

Content goes here...
```

## Import/Export Formats

### Content Import
- **Format**: Markdown with YAML frontmatter
- **Schema**: Validated against Chapter interface
- **Batch Import**: Support for multiple files

### Configuration Export
- **Format**: JavaScript module
- **Purpose**: Backup and version control
- **Example**: Site configuration export

## Performance Considerations

### Build-time Optimizations
- Content validation during build
- Image optimization and lazy loading
- CSS purging for unused utilities
- Bundle splitting by route

### Runtime Optimizations
- No API calls for static data
- Component memoization for expensive renders
- Virtual scrolling for long content lists
- Debounced search and filtering

## Migration Strategy

### v1.0 to v1.1
1. Add new optional fields with defaults
2. Migrate existing content with validation
3. Provide migration scripts if needed

### Breaking Changes
- Version the configuration schema
- Provide migration guide
- Maintain backward compatibility when possible