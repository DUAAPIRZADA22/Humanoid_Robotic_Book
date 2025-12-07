import React from 'react';
import clsx from 'clsx';
import './styles.css';

const outcomes = [
  {
    title: 'Physical AI Foundations',
    description: 'Master the principles governing AI in physical systems and understand embodied intelligence',
    icon: '🧠',
    topics: [
      'AI-robotics integration concepts',
      'Embodiment theory',
      'Physical intelligence paradigms'
    ]
  },
  {
    title: 'Humanoid Robot Design',
    description: 'Learn to design and build humanoid robots from mechanical structures to control systems',
    icon: '🤖',
    topics: [
      'Mechanical design principles',
      'Actuator and sensor integration',
      'Control architecture design'
    ]
  },
  {
    title: 'Machine Learning for Robotics',
    description: 'Train AI models to control physical bodies and interact with the real world',
    icon: '🎯',
    topics: [
      'Reinforcement learning for robotics',
      'Neural network controllers',
      'Simulation-to-real transfer'
    ]
  },
  {
    title: 'Computer Vision & Perception',
    description: 'Enable robots to perceive, understand, and navigate their environment',
    icon: '👁️',
    topics: [
      'Visual perception systems',
      'Sensor fusion techniques',
      '3D environment understanding'
    ]
  },
  {
    title: 'Control Theory & Dynamics',
    description: 'Master mathematical foundations for robot movement, balance, and manipulation',
    icon: '⚡',
    topics: [
      'Kinematics and dynamics',
      'Motion planning algorithms',
      'Balance and stability control'
    ]
  },
  {
    title: 'Real-World Applications',
    description: 'Apply your knowledge to build cutting-edge robotic systems for various industries',
    icon: '🚀',
    topics: [
      'Industrial automation',
      'Service robotics',
      'Research and development'
    ]
  }
];

export default function LearningOutcomes() {
  return (
    <section className="learning-outcomes">
      <div className="container">
        <div className="section-header">
          <h2 className="section-title">What You'll Learn</h2>
          <p className="section-subtitle">
            Master the complete stack of Physical AI and Humanoid Robotics through our comprehensive curriculum
          </p>
        </div>
        <div className="outcomes-grid">
          {outcomes.map((outcome, idx) => (
            <div key={idx} className="outcome-card">
              <div className="outcome-header">
                <div className="outcome-icon">{outcome.icon}</div>
                <h3 className="outcome-title">{outcome.title}</h3>
              </div>
              <p className="outcome-description">{outcome.description}</p>
              <div className="outcome-topics">
                <h4 className="topics-title">Key Topics:</h4>
                <ul className="topics-list">
                  {outcome.topics.map((topic, topicIdx) => (
                    <li key={topicIdx} className="topic-item">
                      <span className="topic-bullet">✓</span>
                      {topic}
                    </li>
                  ))}
                </ul>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}