import React from 'react';
import clsx from 'clsx';
import './styles.css';
import Link from '@docusaurus/Link';

export default function Hero({title, subtitle, ctaText, ctaHref}) {
  return (
    <div className="hero">
      <div className="hero-gradient"></div>
      <div className="hero-particles"></div>
      <div className="container">
        <div className="hero-content">
          <div className="hero-badge">
            <span className="badge-text">🎓 Comprehensive Course</span>
          </div>
          <h1 className="hero-title">{title}</h1>
          <p className="hero-subtitle">{subtitle}</p>
          <div className="hero-buttons">
            <Link
              to={ctaHref}
              className="button button--primary button--lg hero-button primary-cta"
            >
              <span className="button-content">
                {ctaText}
                <svg className="button-arrow" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M5 12h14m-7-7l7 7-7 7"/>
                </svg>
              </span>
            </Link>
            <Link
              to="https://github.com/DUAAPIRZADA22/Humanoid_Robotic_Book"
              className="button button--secondary button--lg hero-button secondary-cta"
            >
              <svg className="github-icon" width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
              </svg>
              View on GitHub
            </Link>
          </div>
          <div className="hero-stats">
            <div className="stat-item">
              <span className="stat-number">4</span>
              <span className="stat-label">Modules</span>
            </div>
            <div className="stat-item">
              <span className="stat-number">12+</span>
              <span className="stat-label">Chapters</span>
            </div>
            <div className="stat-item">
              <span className="stat-number">100+</span>
              <span className="stat-label">Examples</span>
            </div>
          </div>
        </div>
        <div className="hero-animation">
          <div className="hero-card-container">
            <div className="hero-card book-card">
              <div className="card-header">
                <div className="card-icon">📚</div>
                <span className="card-badge">Module 1</span>
              </div>
              <div className="card-content">
                <h3>Foundations</h3>
                <p>Kinematics & Setup</p>
              </div>
            </div>
            <div className="hero-card ai-card">
              <div className="card-header">
                <div className="card-icon">🧠</div>
                <span className="card-badge">Module 2</span>
              </div>
              <div className="card-content">
                <h3>Perception & Control</h3>
                <p>ROS2 & Simulation</p>
              </div>
            </div>
            <div className="hero-card robot-card">
              <div className="card-header">
                <div className="card-icon">🤖</div>
                <span className="card-badge">Module 3</span>
              </div>
              <div className="card-content">
                <h3>AI Integration</h3>
                <p>Machine Learning</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}