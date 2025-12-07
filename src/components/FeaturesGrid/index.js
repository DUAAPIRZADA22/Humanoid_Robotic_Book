import React from 'react';
import clsx from 'clsx';
import './styles.css';

const features = [
  {
    title: 'AI-Native Curriculum',
    description: 'Learn how AI thinking transforms robotics education with practical, hands-on examples.',
    icon: '🎯',
  },
  {
    title: 'Embodied Intelligence',
    description: 'Master the intersection of physical robotics and AI through comprehensive modules.',
    icon: '🤖',
  },
  {
    title: 'Hands-On Projects',
    description: 'Build real humanoid robots and AI systems from scratch with step-by-step guidance.',
    icon: '⚙️',
  },
  {
    title: 'Expert Community',
    description: 'Join a community of learners and experts pushing the boundaries of Physical AI.',
    icon: '👥',
  },
  {
    title: 'Flexible Learning',
    description: 'Learn at your own pace with self-paced modules and comprehensive documentation.',
    icon: '📚',
  },
  {
    title: 'Future-Ready Skills',
    description: 'Acquire cutting-edge skills in robotics, AI, and embodied intelligence.',
    icon: '🚀',
  },
];

export default function FeaturesGrid() {
  return (
    <section className="features">
      <div className="container">
        <div className="header">
          <h2 className="title">Why Choose This Course?</h2>
          <p className="subtitle">
            Discover the future of robotics through our comprehensive, AI-driven curriculum
          </p>
        </div>
        <div className="grid">
          {features.map((feature, idx) => (
            <div key={idx} className="card feature-card">
              <div className="feature-icon">{feature.icon}</div>
              <h3 className="feature-title">{feature.title}</h3>
              <p className="feature-description">{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}