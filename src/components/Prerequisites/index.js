import React from 'react';
import clsx from 'clsx';
import './styles.css';

const prerequisites = [
  {
    title: 'Programming Skills',
    required: true,
    items: [
      { name: 'Python Programming', level: 'Intermediate', description: 'Comfortable with functions, classes, and data structures' },
      { name: 'Basic C++ Knowledge', level: 'Beginner', description: 'Understanding of syntax and basic concepts (helpful)' },
      { name: 'Linux Command Line', level: 'Basic', description: 'Familiarity with terminal commands and file operations' }
    ]
  },
  {
    title: 'Mathematics',
    required: true,
    items: [
      { name: 'Linear Algebra', level: 'Intermediate', description: 'Vectors, matrices, transformations, and eigenvalues' },
      { name: 'Calculus', level: 'Intermediate', description: 'Derivatives, integrals, and basic differential equations' },
      { name: 'Probability & Statistics', level: 'Basic', description: 'Understanding of probability distributions and basic statistics' }
    ]
  },
  {
    title: 'Machine Learning',
    required: false,
    items: [
      { name: 'Basic ML Concepts', level: 'Helpful', description: 'Understanding of supervised/unsupervised learning' },
      { name: 'Neural Networks', level: 'Helpful', description: 'Basic knowledge of neural network architectures' },
      { name: 'Computer Vision', level: 'Optional', description: 'Familiarity with image processing concepts' }
    ]
  },
  {
    title: 'Technical Setup',
    required: true,
    items: [
      { name: 'Computer Requirements', level: 'Required', description: 'Modern computer with at least 16GB RAM' },
      { name: 'Internet Connection', level: 'Required', description: 'Stable connection for downloading tools and datasets' },
      { name: 'Docker Installation', level: 'Helpful', description: 'Basic understanding of containerization' }
    ]
  }
];

export default function Prerequisites() {
  return (
    <section className="prerequisites">
      <div className="container">
        <div className="section-header">
          <h2 className="section-title">Prerequisites</h2>
          <p className="section-subtitle">
            Everything you need to get started on your Physical AI journey
          </p>
        </div>

        <div className="prerequisites-grid">
          {prerequisites.map((category, idx) => (
            <div key={idx} className={`prerequisite-category ${category.required ? 'required' : 'optional'}`}>
              <div className="category-header">
                <h3 className="category-title">{category.title}</h3>
                <span className={`category-badge ${category.required ? 'badge-required' : 'badge-optional'}`}>
                  {category.required ? 'Required' : 'Recommended'}
                </span>
              </div>
              <div className="category-items">
                {category.items.map((item, itemIdx) => (
                  <div key={itemIdx} className="prerequisite-item">
                    <div className="item-header">
                      <h4 className="item-name">{item.name}</h4>
                      <span className={`item-level ${getLevelClass(item.level)}`}>
                        {item.level}
                      </span>
                    </div>
                    <p className="item-description">{item.description}</p>
                  </div>
                ))}
              </div>
            </div>
          ))}
        </div>

        <div className="getting-started">
          <div className="started-content">
            <h3 className="started-title">Don't Have All Prerequisites?</h3>
            <p className="started-description">
              No worries! We provide comprehensive setup guides and learning resources to help you get up to speed.
              Our course includes detailed tutorials for all required tools and concepts.
            </p>
            <div className="started-actions">
              <a href="/docs/development-setup" className="button button--primary button--lg">
                View Setup Guide
              </a>
              <a href="/docs/module-1/setup" className="button button--secondary button--lg">
                Learning Resources
              </a>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function getLevelClass(level) {
  switch (level.toLowerCase()) {
    case 'required':
      return 'level-required';
    case 'intermediate':
      return 'level-intermediate';
    case 'basic':
    case 'beginner':
      return 'level-basic';
    case 'helpful':
    case 'optional':
      return 'level-optional';
    default:
      return 'level-default';
  }
}