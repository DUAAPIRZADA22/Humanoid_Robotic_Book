# Chapter Content API Specification

**Version**: 1.0.0
**Last Updated**: 2025-12-06

## Overview

This specification defines the interface for creating and managing enhanced textbook chapters with rich, descriptive content and professional diagrams.

## Endpoints

### GET /chapters/template

Retrieve the standard chapter template for content creation.

**Query Parameters**:
- `type` (optional): Chapter type (ai-brain | advanced-humanoids)
- `complexity` (optional): Content complexity (beginner | intermediate | advanced)

**Response**:
```json
{
  "template": {
    "frontmatter": {
      "title": "Chapter X: [Title]",
      "sidebar_position": 40,
      "description": "[Brief description]",
      "tags": ["tag1", "tag2"],
      "authors": ["author"]
    },
    "structure": [
      "Learning Objectives",
      "Prerequisites",
      "Introduction",
      "Core Concepts",
      "Implementation",
      "In Practice",
      "Advanced Topics",
      "Knowledge Check"
    ],
    "requirements": {
      "min_words": 2000,
      "min_diagrams": 3,
      "min_code_examples": 3,
      "min_knowledge_checks": 5
    }
  }
}
```

### POST /chapters/validate

Validate chapter content against quality standards.

**Request Body**:
```json
{
  "chapter_path": "docs/module-2/part4-ai-brain/chapter-8-isaac-sim.md",
  "validation_options": {
    "check_word_count": true,
    "validate_diagrams": true,
    "check_code_examples": true,
    "validate_links": true,
    "accessibility_check": true
  }
}
```

**Response**:
```json
{
  "validation_result": "PASS",
  "metrics": {
    "word_count": 2450,
    "diagram_count": 4,
    "code_examples": 5,
    "knowledge_checks": 7,
    "readability_score": 8.5
  },
  "issues": [],
  "warnings": [
    {
      "type": "STYLE",
      "message": "Consider adding more real-world examples in Section 3.2"
    }
  ]
}
```

### GET /diagrams/types

Retrieve supported diagram types and templates.

**Response**:
```json
{
  "supported_types": [
    {
      "type": "flowchart",
      "description": "Process flows and decision trees",
      "template": "flowchart TD\n    A[Start] --> B[Process]"
    },
    {
      "type": "sequence",
      "description": "System interactions over time",
      "template": "sequenceDiagram\n    participant A\n    participant B\n    A->>B: Message"
    },
    {
      "type": "class",
      "description": "Software architecture",
      "template": "classDiagram\n    class ClassName\n    ClassName: +method()"
    },
    {
      "type": "state",
      "description": "State machines",
      "template": "stateDiagram-v2\n    [*] --> State1"
    },
    {
      "type": "gantt",
      "description": "Project timelines",
      "template": "gantt\n    title Timeline\n    section Section\n    Task: a1, 2024-01-01, 30d"
    }
  ]
}
```

### POST /diagrams/generate

Generate diagram from description using AI assistance.

**Request Body**:
```json
{
  "description": "Show the ROS 2 node interaction with Isaac Sim for sensor data collection",
  "type": "sequence",
  "elements": [
    "ROS 2 Node",
    "Isaac Sim",
    "Sensors",
    "Data Logger"
  ]
}
```

**Response**:
```json
{
  "mermaid_code": "sequenceDiagram\n    participant ROS as ROS 2 Node\n    participant ISAAC as Isaac Sim\n    participant SENSOR as Sensors\n    participant DATA as Data Logger\n\n    ROS->>ISAAC: Initialize simulation\n    ISAAC->>SENSOR: Activate sensors\n    SENSOR->>DATA: Stream data\n    DATA->>ROS: Publish to topics",
  "complexity": "medium",
  "estimated_render_time": "0.5s"
}
```

## Data Models

### Chapter
```yaml
Chapter:
  id: string
  metadata:
    title: string
    subtitle: string
    authors: array[string]
    version: string
    created_at: datetime
    updated_at: datetime
  content:
    sections: array[Section]
    diagrams: array[Diagram]
    code_examples: array[CodeExample]
    knowledge_checks: array[KnowledgeCheck]
  metrics:
    word_count: number
    reading_time: number
    difficulty_level: number
```

### Diagram
```yaml
Diagram:
  id: string
  type: enum[flowchart, sequence, class, state, gantt, pie]
  mermaid_code: string
  title: string
  caption: string
  complexity: enum[low, medium, high]
  referenced_in: array[string]
```

### CodeExample
```yaml
CodeExample:
  id: string
  language: enum[python, cpp, yaml, bash]
  title: string
  description: string
  code: string
  has_documentation: boolean
  has_error_handling: boolean
  has_tests: boolean
```

## Validation Rules

### Content Quality Gates
1. **Word Count**: Must be >= 2000 words
2. **Diagrams**: Minimum 3, maximum 6 per chapter
3. **Code Examples**: Technical chapters require >= 3
4. **Knowledge Checks**: Minimum 5 per chapter
5. **Readability**: Flesch-Kincaid score between 7-12

### Technical Requirements
1. **Mermaid Syntax**: All diagrams must be valid
2. **Code Execution**: All examples must run
3. **Links**: All internal links must resolve
4. **Images**: Alt text required for all images
5. **Accessibility**: WCAG 2.1 AA compliance

### Style Guidelines
1. **Headings**: Consistent use of title case
2. **Code Blocks**: Syntax highlighting with line numbers
3. **Callouts**: Use Docusaurus admonitions
4. **Cross-references**: Link to related content
5. **Glossary**: Define technical terms on first use

## Error Responses

### 400 Bad Request
```json
{
  "error": "INVALID_CHAPTER_STRUCTURE",
  "message": "Chapter must include all required sections",
  "details": {
    "missing_sections": ["Advanced Topics"]
  }
}
```

### 422 Unprocessable Entity
```json
{
  "error": "VALIDATION_FAILED",
  "message": "Chapter content does not meet quality standards",
  "details": {
    "word_count": {
      "actual": 1500,
      "required": 2000
    },
    "diagrams": {
      "actual": 1,
      "required": 3
    }
  }
}
```

## Rate Limiting

- Content validation: 10 requests per minute
- Diagram generation: 5 requests per minute
- Template retrieval: 100 requests per minute

## Versioning

API version is included in the URL path: `/v1/chapters/...`

Backward compatibility is maintained for:
- Existing chapter formats
- Legacy diagram types
- Previous validation rules