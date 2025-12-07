# Book Content Writer Style Guide

## Tone and Voice

### Professional Minimalist Style
- **Clarity over cleverness**: Write directly and simply
- **Economical expression**: Every word should serve a purpose
- **Confident without arrogance**: State facts clearly; avoid opinionated language
- **Helpful, not condescending**: Assume intelligent readers who are new to this topic

### Voice Guidelines
- Use active voice ("We configure the system" not "The system is configured")
- Keep sentences under 25 words when possible
- Use present tense for general truths and future tense for instructions
- Address reader as "you" when giving instructions

## Negative Constraints (Strictly Forbidden)

### NEVER Use These Buzzwords
- delve
- revolutionize
- tapestry
- journey
- unleash
- empower (unless referring to specific permission systems)
- leverage (as a verb)
- synergy
- paradigm shift
- game-changing
- robust (unless describing statistical methods)
- seamless
- intuitive (unless discussing UI/UX research)
- cutting-edge
- state-of-the-art

### Avoid These Patterns
- Metaphors that don't add clarity
- Rhetorical questions
- Overly casual language ("Hey folks!", "Let's dive in!")
- Academic jargon without definition
- Excessive exclamation marks
- Emojis in formal documentation

## Content Structure

### Chapter Template
```markdown
---
title: "Chapter Title"
description: "One-sentence summary for SEO and previews"
sidebar_label: "Short Title"
---

# Chapter Title

[Hook sentence that establishes relevance and context]

In this chapter, you will learn:
- [Key learning outcome 1]
- [Key learning outcome 2]
- [Key learning outcome 3]

## Section 1: Clear, Action-Oriented Title

[2-3 paragraphs introducing the concept]

### Subsection: Specific Topic

[Explain with examples]

```language
[code example with full comments]
```

[Explanation of what the code does]

:::tip Practical Application
[Real-world use case or best practice]
:::

## Section 2: Next Logical Topic

[Continue with progressive complexity]
```

### Heading Hierarchy
- H1: Chapter title (only one per file)
- H2: Main sections (3-7 per chapter)
- H3: Subsections (2-5 per H2)
- Never skip levels (H1 → H3)

## Code Examples

### Requirements
- **Complete and runnable**: Code should work when copied
- **Fully commented**: Explain non-obvious parts
- **Language specified**: Always use syntax highlighting
- **Context provided**: Show necessary imports/setup
- **Error handling**: Include for production-ready examples

### Example Format
```typescript
// Import required modules
import { Module } from 'package-name';

// Initialize the service with configuration
const service = new Module({
  apiKey: process.env.API_KEY, // Never hardcode credentials
  timeout: 5000, // 5 second timeout for requests
});

// Main function to demonstrate usage
async function main() {
  try {
    const result = await service.getData();
    console.log('Success:', result);
  } catch (error) {
    console.error('Operation failed:', error.message);
  }
}
```

## Docusaurus Features

### Admonitions (Use Sparingly)
```markdown
:::note
Neutral information that provides context but isn't critical
:::

:::tip
Actionable advice that saves time or prevents common mistakes
:::

:::warning
Critical information about potential risks or breaking changes
:::

:::danger
Information that could lead to data loss or security issues
:::
```

### Tabs for Multi-Option Examples
```jsx
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
  <TabItem value="npm" label="npm">
    ```bash
    npm install package-name
    ```
  </TabItem>
  <TabItem value="yarn" label="Yarn">
    ```bash
    yarn add package-name
    ```
  </TabItem>
</Tabs>
```

## Writing Mechanics

### Emphasis and Formatting
- **Bold** for key terms on first use and UI elements
- `Code ticks` for variables, file names, and inline commands
- *Italics* only for technical terms being defined
- `Monospace` for file paths and URLs

### Lists
- Use numbered lists for sequences or steps
- Use bulleted lists for non-ordered items
- Keep list items parallel in structure
- Maximum 7-9 items per list

### Links
- Descriptive text, not "click here"
- Internal links use relative paths
- External links include helpful context
- Check all links work in preview

## Accessibility

### Images
- Always include descriptive alt text
- If complex, include long description in caption
- Don't rely on color alone for meaning

### Code
- Use high-contrast syntax highlighting
- Ensure font size is readable
- Avoid color-coding information without text labels

## Quality Checklist Before Publishing

### Content Review
- [ ] No forbidden buzzwords used
- [ ] All code examples tested and work
- [ ] Headings follow proper hierarchy
- [ ] All links functional and appropriate
- [ ] Technical accuracy verified

### Style Review
- [ ] Consistent tone throughout
- [ ] No overly casual or academic language
- [ ] Proper use of Docusaurus components
- [ ] Adequate use of white space
- [ ] Clear transitions between sections

### Completeness
- [ ] Introduction establishes relevance
- [ ] Each section builds on previous
- [ ] Summary or next steps provided
- [ ] Prerequisites clearly stated
- [ ] Learning outcomes achieved