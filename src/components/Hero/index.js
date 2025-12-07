import React from 'react';
import clsx from 'clsx';
import './styles.css';
import Link from '@docusaurus/Link';

export default function Hero({title, subtitle, ctaText, ctaHref}) {
  return (
    <div className="hero">
      <div className="hero-gradient"></div>
      <div className="container">
        <div className="hero-content">
          <h1 className="hero-title">{title}</h1>
          <p className="hero-subtitle">{subtitle}</p>
          <div className="hero-buttons">
            <Link
              to={ctaHref}
              className="button button--primary button--lg hero-button"
            >
              {ctaText}
            </Link>
            <Link
              to="https://github.com/example/humanoid_robotic_book"
              className="button button--secondary button--lg hero-button"
            >
              View on GitHub
            </Link>
          </div>
        </div>
        <div className="hero-animation">
          <div className="floating-card">
            <div className="card-icon">🤖</div>
            <div className="card-title">Humanoid Robots</div>
          </div>
          <div className="floating-card" style={{animationDelay: '0.5s'}}>
            <div className="card-icon">🧠</div>
            <div className="card-title">Physical AI</div>
          </div>
          <div className="floating-card" style={{animationDelay: '1s'}}>
            <div className="card-icon">⚡</div>
            <div className="card-title">Embodied Intelligence</div>
          </div>
        </div>
      </div>
    </div>
  );
}