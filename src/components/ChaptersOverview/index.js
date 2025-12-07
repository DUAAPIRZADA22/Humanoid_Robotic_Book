import React from 'react';
import clsx from 'clsx';
import './styles.css';
import Link from '@docusaurus/Link';

const chaptersOverview = [
  {
    module: 1,
    title: 'Foundations',
    subtitle: 'Building the basics of Physical AI',
    chapters: [
      {
        number: 1,
        title: 'Introduction to Physical AI',
        description: 'Understanding the intersection of AI and robotics',
        duration: '2 hours',
        difficulty: 'Beginner',
        link: '/docs/module-1/foundations'
      },
      {
        number: 2,
        title: 'Development Setup',
        description: 'Installing and configuring your robotics development environment',
        duration: '3 hours',
        difficulty: 'Beginner',
        link: '/docs/module-1/setup'
      },
      {
        number: 3,
        title: 'Kinematics and Movement',
        description: 'Mathematical foundations of robot motion',
        duration: '4 hours',
        difficulty: 'Intermediate',
        link: '/docs/module-1/kinematics'
      }
    ],
    totalDuration: '9 hours',
    icon: '🏗️'
  },
  {
    module: 2,
    title: 'Physical AI & Humanoid Robotics',
    subtitle: 'Deep dive into perception and systems',
    chapters: [
      {
        number: 4,
        title: 'Physical AI Foundations',
        description: 'Core concepts and principles of embodied intelligence',
        duration: '3 hours',
        difficulty: 'Intermediate',
        link: '/docs/module-2/part1-foundations/chapter-1-introduction'
      },
      {
        number: 5,
        title: 'Sensors and Perception',
        description: 'Understanding robot sensing capabilities',
        duration: '4 hours',
        difficulty: 'Intermediate',
        link: '/docs/module-2/part1-foundations/chapter-2-sensors-perception'
      },
      {
        number: 6,
        title: 'ROS 2 Architecture',
        description: 'Building robust robotic systems with ROS 2',
        duration: '5 hours',
        difficulty: 'Intermediate',
        link: '/docs/module-2/part2-nervous-system/chapter-3-ros2-architecture'
      },
      {
        number: 7,
        title: 'Building ROS 2 Nodes',
        description: 'Creating modular robotics software components',
        duration: '4 hours',
        difficulty: 'Intermediate',
        link: '/docs/module-2/part2-nervous-system/chapter-4-building-ros2-nodes-python'
      },
      {
        number: 8,
        title: 'Launch Systems & Parameters',
        description: 'Managing complex robot configurations',
        duration: '3 hours',
        difficulty: 'Intermediate',
        link: '/docs/module-2/part2-nervous-system/chapter-5-launch-systems-parameter-management'
      },
      {
        number: 9,
        title: 'Gazebo Simulation',
        description: 'Creating realistic robot simulations',
        duration: '5 hours',
        difficulty: 'Advanced',
        link: '/docs/module-2/part3-digital-twin/chapter-6-gazebo-simulation'
      },
      {
        number: 10,
        title: 'Physics Simulation',
        description: 'Advanced simulation with Unity and physics engines',
        duration: '4 hours',
        difficulty: 'Advanced',
        link: '/docs/module-2/part3-digital-twin/chapter-7-physics-simulation-unity'
      },
      {
        number: 11,
        title: 'NVIDIA Isaac Sim',
        description: 'Professional-grade AI robotics simulation',
        duration: '6 hours',
        difficulty: 'Advanced',
        link: '/docs/module-2/part4-ai-brain/chapter-8-nvidia-isaac-sim'
      }
    ],
    totalDuration: '34 hours',
    icon: '🧠'
  },
  {
    module: 3,
    title: 'Control',
    subtitle: 'Mastering robot control systems',
    chapters: [
      {
        number: 12,
        title: 'Control Systems Fundamentals',
        description: 'Mathematical foundations of robot control',
        duration: '4 hours',
        difficulty: 'Intermediate',
        link: '/docs/module-3/control'
      }
    ],
    totalDuration: '4 hours',
    icon: '🎮'
  },
  {
    module: 4,
    title: 'Applications',
    subtitle: 'Real-world implementations',
    chapters: [
      {
        number: 13,
        title: 'Real-world Applications',
        description: 'Applying Physical AI in industry and research',
        duration: '3 hours',
        difficulty: 'Advanced',
        link: '/docs/module-4/applications'
      }
    ],
    totalDuration: '3 hours',
    icon: '🚀'
  }
];

const bookInfo = {
  title: 'Physical AI & Humanoid Robotics Book',
  description: 'Get the complete reference guide with advanced concepts, detailed examples, and practical projects.',
  features: [
    '500+ pages of in-depth content',
    '25+ hands-on projects',
    'Complete code examples',
    'Advanced AI algorithms',
    'Industry case studies',
    'Lifetime updates'
  ],
  price: '$49.99',
  badge: 'Coming Soon'
};

export default function ChaptersOverview() {
  return (
    <section className="chapters-overview">
      <div className="container">
        <div className="section-header">
          <h2 className="section-title">See All Chapters</h2>
          <p className="section-subtitle">
            Explore our comprehensive curriculum with 13 chapters covering everything from basics to advanced topics
          </p>
        </div>

        <div className="modules-container">
          {chaptersOverview.map((module, idx) => (
            <div key={idx} className="module-card">
              <div className="module-header">
                <div className="module-icon">{module.icon}</div>
                <div className="module-info">
                  <h3 className="module-title">Module {module.module}: {module.title}</h3>
                  <p className="module-subtitle">{module.subtitle}</p>
                  <div className="module-meta">
                    <span className="module-duration">{module.totalDuration}</span>
                    <span className="module-chapters-count">{module.chapters.length} chapters</span>
                  </div>
                </div>
              </div>

              <div className="chapters-list">
                {module.chapters.map((chapter, chapterIdx) => (
                  <div key={chapterIdx} className="chapter-item">
                    <div className="chapter-number">{chapter.number}</div>
                    <div className="chapter-content">
                      <div className="chapter-header">
                        <h4 className="chapter-title">{chapter.title}</h4>
                        <div className="chapter-meta">
                          <span className={`difficulty-badge ${getDifficultyClass(chapter.difficulty)}`}>
                            {chapter.difficulty}
                          </span>
                          <span className="chapter-duration">{chapter.duration}</span>
                        </div>
                      </div>
                      <p className="chapter-description">{chapter.description}</p>
                      <Link
                        to={chapter.link}
                        className="chapter-link"
                      >
                        Start Chapter →
                      </Link>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          ))}
        </div>

        <div className="book-section">
          <div className="book-content">
            <div className="book-info">
              <div className="book-header">
                <h3 className="book-title">{bookInfo.title}</h3>
                <span className="book-badge">{bookInfo.badge}</span>
              </div>
              <p className="book-description">{bookInfo.description}</p>

              <div className="book-features">
                {bookInfo.features.map((feature, idx) => (
                  <div key={idx} className="feature-item">
                    <span className="feature-icon">✓</span>
                    <span className="feature-text">{feature}</span>
                  </div>
                ))}
              </div>

              <div className="book-actions">
                <button className="button button--primary button--lg book-button" disabled>
                  Pre-order Now - {bookInfo.price}
                </button>
                <button className="button button--secondary button--lg">
                  Get Free Sample Chapter
                </button>
              </div>
            </div>

            <div className="book-visual">
              <div className="book-placeholder">
                <div className="book-cover">
                  <div className="book-icon">📚</div>
                  <div className="book-cover-text">Physical AI &<br/>Humanoid Robotics</div>
                  <div className="book-cover-subtitle">Complete Reference Guide</div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function getDifficultyClass(difficulty) {
  switch (difficulty.toLowerCase()) {
    case 'beginner':
      return 'difficulty-beginner';
    case 'intermediate':
      return 'difficulty-intermediate';
    case 'advanced':
      return 'difficulty-advanced';
    default:
      return 'difficulty-default';
  }
}