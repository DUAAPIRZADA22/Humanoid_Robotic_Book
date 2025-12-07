import React from 'react';
import clsx from 'clsx';
import './styles.css';
import Link from '@docusaurus/Link';

const modules = [
  {
    id: 1,
    title: 'Foundations',
    description: 'Introduction to Physical AI, robotics basics, and essential concepts',
    icon: '🏗️',
    duration: '2 weeks',
    chapters: ['Introduction to Physical AI', 'Robotics Fundamentals', 'AI and Embodiment'],
    link: '/docs/module-1/foundations',
  },
  {
    id: 2,
    title: 'Perception & Sensing',
    description: 'Computer vision, sensor fusion, and environmental understanding',
    icon: '👁️',
    duration: '3 weeks',
    chapters: ['Computer Vision Basics', 'Sensor Fusion', 'Spatial Awareness'],
    link: '/docs/module-2/perception',
  },
  {
    id: 3,
    title: 'Control & Movement',
    description: 'Motion planning, control systems, and locomotion strategies',
    icon: '🎮',
    duration: '3 weeks',
    chapters: ['Kinematics', 'Motion Planning', 'Balance and Stability'],
    link: '/docs/module-3/control',
  },
  {
    id: 4,
    title: 'Advanced Integration',
    description: 'Putting it all together with advanced AI and real-world applications',
    icon: '🚀',
    duration: '4 weeks',
    chapters: ['Neural Networks', 'Reinforcement Learning', 'Real-world Deployment'],
    link: '/docs/module-4/applications',
  },
];

export default function CourseTimeline() {
  return (
    <section className="timeline">
      <div className="container">
        <div className="header">
          <h2 className="title">Course Journey</h2>
          <p className="subtitle">
            Master Physical AI through our structured 4-module curriculum
          </p>
        </div>
        <div className="timeline-container">
          {modules.map((module, idx) => (
            <div key={module.id} className="timeline-item">
              <div className="timeline-content">
                <div className="module-header">
                  <div className="module-icon">{module.icon}</div>
                  <div className="module-info">
                    <h3 className="module-title">
                      Module {module.id}: {module.title}
                    </h3>
                    <p className="module-duration">{module.duration}</p>
                  </div>
                </div>
                <p className="module-description">{module.description}</p>
                <div className="module-chapters">
                  <h4 className="chapters-title">What you'll learn:</h4>
                  <ul className="chapters-list">
                    {module.chapters.map((chapter, chapterIdx) => (
                      <li key={chapterIdx} className="chapter-item">
                        {chapter}
                      </li>
                    ))}
                  </ul>
                </div>
                <Link
                  to={module.link}
                  className="button button--primary module-button"
                >
                  Start Module {module.id}
                </Link>
              </div>
              <div className="timeline-connector"></div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}