# Data Model: Enhanced Textbook Content Structure

**Date**: 2025-12-06
**Purpose**: Define data structures for enhanced, descriptive textbook chapters with diagrams

## Core Entities

### 1. Chapter

```yaml
Chapter:
  metadata:
    id: string               # e.g., "chapter-8-nvidia-isaac-sim"
    title: string            # Human-readable title
    subtitle: string         # Descriptive subtitle
    authors: array[string]   # List of authors/contributors
    version: string          # Semantic version
    last_updated: datetime   # Last modification date
    estimated_read_time: number # Minutes to read

  content_structure:
    word_count: number       # Target: 2000+
    sections: array[Section] # Logical content divisions
    code_examples: array[CodeExample] # Working examples
    diagrams: array[Diagram] # Mermaid.js diagrams
    knowledge_checks: array[KnowledgeCheck] # Assessments

  learning_outcomes:
    objectives: array[string] # Learning objectives
    prerequisites: array[string] # Required knowledge
    skills_gained: array[string] # Acquired skills
```

### 2. Section

```yaml
Section:
  id: string               # Unique section identifier
  title: string            # Section title
  type: enum               # concept | implementation | example | advanced
  content_markdown: string # Section content
  subsections: array[Section] # Nested structure

  enhancement_metadata:
    difficulty_level: enum  # beginner | intermediate | advanced
    estimated_time: number  # Minutes
    practical_value: number # 1-10 scale
    theoretical_depth: number # 1-10 scale

  cross_references:
    related_sections: array[string] # Internal links
    external_resources: array[Resource] # External references
```

### 3. CodeExample

```yaml
CodeExample:
  id: string               # Unique identifier
  title: string            # Example title
  description: string      # Purpose and context
  language: enum           # python | cpp | yaml | bash

  code_content:
    main_file: string      # Primary code
    supporting_files: array[string] # Helper files
    dependencies: array[Dependency] # Required packages

  documentation:
    docstring: string      # Comprehensive description
    parameters: array[Parameter] # Function parameters
    returns: ReturnInfo    # Return value information
    examples: array[string] # Usage examples

  quality_attributes:
    has_error_handling: boolean
    has_type_hints: boolean
    has_tests: boolean
    platform_compatible: array[string] # Supported platforms
```

### 4. Diagram

```yaml
Diagram:
  id: string               # Unique identifier
  title: string            # Diagram title
  type: enum               # flowchart | sequence | class | state | gantt | pie

  visual_specification:
    mermaid_code: string   # Diagram definition
    caption: string        # Detailed explanation
    width: number          # Suggested width
    height: number         # Suggested height

  content_integration:
    referenced_sections: array[string] # Where it's used
    explanation_text: string # Detailed caption
    interactive_elements: array[string] # Clickable areas

  accessibility:
    alt_text: string       # Screen reader description
    high_contrast_version: boolean # Colorblind friendly
```

### 5. KnowledgeCheck

```yaml
KnowledgeCheck:
  id: string               # Unique identifier
  type: enum               # multiple_choice | short_answer | practical | discussion

  question_content:
    question: string       # Question text
    context: string        # Additional context
    hints: array[string]   # Optional hints

  answer_specification:
    correct_answer: string # For objective questions
    rubric: string        # For subjective questions
    explanation: string   # Why it's correct

  metadata:
    difficulty: enum       # easy | medium | hard
    concept_covered: string # Related concept
    estimated_time: number # Minutes to complete
```

## Enhanced Content Attributes

### Descriptiveness Metrics

```yaml
ContentMetrics:
  readability_score: number # Flesch-Kincaid score
  technical_depth: number   # 1-10 scale
  practical_examples: number # Count of real-world examples
  concept_coverage: array[string] # List of concepts covered

  descriptive_elements:
    real_world_applications: array[Application]
    case_studies: array[CaseStudy]
    historical_context: string # Background information
    future_implications: string # Forward-looking content
```

### Professional Presentation

```yaml
PresentationAttributes:
  visual_hierarchy:
    heading_levels: number # Depth of heading structure
    callouts_used: array[enum] # note | tip | warning | danger
    emphasis_patterns: array[string] # Bold/italic usage guidelines

  navigation_features:
    quick_links: array[QuickLink] # Section navigation
    breadcrumbs: boolean # Show navigation path
    progress_indicator: boolean # Show reading progress

  accessibility_features:
    alt_text_complete: boolean # All images have descriptions
    keyboard_navigation: boolean # Full keyboard access
    screen_reader_optimized: boolean # ARIA labels, etc.
```

## Validation Rules

### Chapter Quality Checks

1. **Minimum Requirements**:
   - Word count >= 2000
   - At least 3 diagrams per chapter
   - At least 5 knowledge checks
   - At least 3 code examples for technical chapters

2. **Descriptiveness Validation**:
   - Each concept must have real-world application example
   - Code examples must include practical context
   - Technical terms must be defined on first use
   - Cross-references must exist for related concepts

3. **Diagram Standards**:
   - All diagrams must have captions
   - Mermaid syntax must be valid
   - Diagrams must be referenced in text
   - Complex diagrams need step-by-step explanations

### Content Consistency

```yaml
ConsistencyRules:
  terminology:
    glossary_terms: array[string] # Standardized terms
    abbreviation_definitions: object # Term -> expansion

  formatting:
    code_block_style: enum # Consistent highlighting
    heading_convention: string # Title Case vs. sentence case
    link_format: string # Consistent link text

  quality_gates:
    spell_check: boolean # Must pass spell check
    grammar_check: boolean # Must pass grammar check
    link_validation: boolean # All links must work
```