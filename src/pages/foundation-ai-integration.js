import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './foundation-ai-integration.module.css';
import ChatWidget from '@site/src/components/ChatWidget';

export default function FoundationAIIntegration() {
  return (
    <Layout
      title="Foundation AI Integration - Physical AI & Humanoid Robotics"
      description="Explore the integration of artificial intelligence with humanoid robotics for intelligent autonomous systems">
      <div className={styles.pageContainer}>
        {/* Hero Section */}
        <section className={styles.hero}>
          <div className={styles.heroContent}>
            <h1>Foundation AI Integration</h1>
            <p>Explore the cutting-edge intersection of artificial intelligence and humanoid robotics, where intelligent algorithms bring machines to life.</p>
            <div className={styles.heroButtons}>
              <Link to="/docs/module-3/control" className="button button--primary button--lg">
                Start Learning
              </Link>
              <Link to="/docs/intro" className="button button--secondary button--lg">
                Course Overview
              </Link>
            </div>
          </div>
          <div className={styles.heroAnimation}>
            <div className={styles.animationGrid}>
              <div className={`${styles.animationCard} ${styles.card1}`}>
                <div className={styles.cardIcon}>🧠</div>
                <h3>Machine Learning</h3>
                <p>Neural networks and deep learning for robotics</p>
              </div>
              <div className={`${styles.animationCard} ${styles.card2}`}>
                <div className={styles.cardIcon}>🤖</div>
                <h3>Humanoid Robots</h3>
                <p>Advanced bipedal robots with human-like capabilities</p>
              </div>
              <div className={`${styles.animationCard} ${styles.card3}`}>
                <div className={styles.cardIcon}>⚡</div>
                <h3>Real-time Control</h3>
                <p>Low-latency control systems for smooth movement</p>
              </div>
              <div className={`${styles.animationCard} ${styles.card4}`}>
                <div className={styles.cardIcon}>👁️</div>
                <h3>Computer Vision</h3>
                <p>Visual perception for environment understanding</p>
              </div>
            </div>
          </div>
        </section>

        {/* Main Content Grid */}
        <div className={styles.contentGrid}>
          {/* Left Column - Modules Overview */}
          <section className={styles.modulesSection}>
            <h2>Core AI Integration Modules</h2>
            <div className={styles.moduleCards}>
              <div className={styles.moduleCard}>
                <div className={styles.moduleHeader}>
                  <span className={styles.moduleNumber}>01</span>
                  <h3>Neural Network Foundations</h3>
                </div>
                <p>Build understanding of neural architectures, backpropagation, and optimization techniques specifically designed for robotic control systems.</p>
                <ul className={styles.moduleTopics}>
                  <li>Feedforward Networks for Control</li>
                  <li>Recurrent Networks for Sequences</li>
                  <li>Transformer Architectures</li>
                  <li>Reinforcement Learning Basics</li>
                </ul>
                <Link to="/docs/module-2/part4-ai-brain/chapter-8-nvidia-isaac-sim" className={styles.moduleLink}>
                  Explore Module →
                </Link>
              </div>

              <div className={styles.moduleCard}>
                <div className={styles.moduleHeader}>
                  <span className={styles.moduleNumber}>02</span>
                  <h3>Perception Systems</h3>
                </div>
                <p>Learn how to implement computer vision and sensor fusion systems that enable robots to understand and navigate their environment intelligently.</p>
                <ul className={styles.moduleTopics}>
                  <li>Camera Calibration & Processing</li>
                  <li>LiDAR and Sensor Fusion</li>
                  <li>Object Detection & Tracking</li>
                  <li>SLAM Algorithms</li>
                </ul>
                <Link to="/docs/module-2/part1-foundations/chapter-2-sensors-perception" className={styles.moduleLink}>
                  Explore Module →
                </Link>
              </div>

              <div className={styles.moduleCard}>
                <div className={styles.moduleHeader}>
                  <span className={styles.moduleNumber}>03</span>
                  <h3>Motion Planning & Control</h3>
                </div>
                <p>Master advanced algorithms for planning robot movements and controlling complex kinematic chains with AI-driven optimization.</p>
                <ul className={styles.moduleTopics}>
                  <li>Inverse Kinematics with AI</li>
                  <li>Path Planning Algorithms</li>
                  <li>Optimal Control Theory</li>
                  <li>Adaptive Control Systems</li>
                </ul>
                <Link to="/docs/module-3/control" className={styles.moduleLink}>
                  Explore Module →
                </Link>
              </div>

              <div className={styles.moduleCard}>
                <div className={styles.moduleHeader}>
                  <span className={styles.moduleNumber}>04</span>
                  <h3>Human-Robot Interaction</h3>
                </div>
                <p>Develop natural language processing and gesture recognition systems for seamless communication between humans and humanoid robots.</p>
                <ul className={styles.moduleTopics}>
                  <li>Natural Language Understanding</li>
                  <li>Gesture Recognition</li>
                  <li>Social Robotics Principles</li>
                  <li>Emotion Recognition</li>
                </ul>
                <Link to="/docs/module-4/applications" className={styles.moduleLink}>
                  Explore Module →
                </Link>
              </div>
            </div>
          </section>

          {/* Right Column - Key Technologies */}
          <section className={styles.technologiesSection}>
            <h2>Key Technologies</h2>
            <div className={styles.techGrid}>
              <div className={styles.techItem}>
                <div className={styles.techIcon}>
                  <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                    <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z"/>
                  </svg>
                </div>
                <h4>TensorFlow/PyTorch</h4>
                <p>Deep learning frameworks for neural network implementation</p>
              </div>

              <div className={styles.techItem}>
                <div className={styles.techIcon}>
                  <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                    <path d="M12 2l3.09 6.26L22 9.27l-5 4.87 1.18 6.88L12 17.77l-6.18 3.25L7 14.14 2 9.27l6.91-1.01L12 2z"/>
                  </svg>
                </div>
                <h4>ROS 2</h4>
                <p>Robot Operating System for distributed robot control</p>
              </div>

              <div className={styles.techItem}>
                <div className={styles.techIcon}>
                  <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                    <path d="M9 11H7v2h2v-2zm4 0h-2v2h2v-2zm4 0h-2v2h2v-2zm2-7h-1V2h-2v2H8V2H6v2H5c-1.11 0-1.99.9-1.99 2L3 20c0 1.1.89 2 2 2h14c1.1 0 2-.9 2-2V6c0-1.1-.9-2-2-2zm0 16H5V9h14v11z"/>
                  </svg>
                </div>
                <h4>OpenCV</h4>
                <p>Computer vision library for image processing and analysis</p>
              </div>

              <div className={styles.techItem}>
                <div className={styles.techIcon}>
                  <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                    <path d="M19 3H5c-1.1 0-2 .9-2 2v14c0 1.1.9 2 2 2h14c1.1 0 2-.9 2-2V5c0-1.1-.9-2-2-2zM9 17H7v-7h2v7zm4 0h-2V7h2v10zm4 0h-2v-4h2v4z"/>
                  </svg>
                </div>
                <h4>Panda Robotics</h4>
                <p>Simulation environment for testing robot algorithms</p>
              </div>

              <div className={styles.techItem}>
                <div className={styles.techIcon}>
                  <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                    <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.94-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z"/>
                  </svg>
                </div>
                <h4>Gazebo</h4>
                <p>3D simulation platform for robot development</p>
              </div>

              <div className={styles.techItem}>
                <div className={styles.techIcon}>
                  <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                    <path d="M20 6h-2.18c.11-.31.18-.65.18-1a2.996 2.996 0 0 0-5.5-1.65l-.5.67-.5-.68C10.96 2.54 10.05 2 9 2 7.34 2 6 3.34 6 5c0 .35.07.69.18 1H4c-1.11 0-1.99.89-1.99 2L2 19c0 1.11.89 2 2 2h16c1.11 0 2-.89 2-2V8c0-1.11-.89-2-2-2zm-5-2c.55 0 1 .45 1 1s-.45 1-1 1-1-.45-1-1 .45-1 1-1zM9 4c.55 0 1 .45 1 1s-.45 1-1 1-1-.45-1-1 .45-1 1-1z"/>
                  </svg>
                </div>
                <h4>Docker</h4>
                <p>Containerization for reproducible AI environments</p>
              </div>
            </div>

            {/* Quick Start Card */}
            <div className={styles.quickStartCard}>
              <h3>Ready to Start?</h3>
              <p>Begin your journey into AI-powered humanoid robotics with our comprehensive guide.</p>
              <Link to="/docs/module-1/setup" className="button button--primary button--lg">
                Quick Start Guide
              </Link>
            </div>
          </section>
        </div>

        {/* Learning Path */}
        <section className={styles.learningPath}>
          <h2>Learning Path</h2>
          <div className={styles.pathContainer}>
            <div className={styles.pathStep}>
              <div className={styles.stepNumber}>1</div>
              <div className={styles.stepContent}>
                <h3>Foundation Concepts</h3>
                <p>Understand the mathematical and theoretical foundations of AI and robotics</p>
              </div>
            </div>
            <div className={styles.pathStep}>
              <div className={styles.stepNumber}>2</div>
              <div className={styles.stepContent}>
                <h3>Practical Implementation</h3>
                <p>Build hands-on projects using real robots and simulation environments</p>
              </div>
            </div>
            <div className={styles.pathStep}>
              <div className={styles.stepNumber}>3</div>
              <div className={styles.stepContent}>
                <h3>Advanced Applications</h3>
                <p>Develop sophisticated AI systems for complex humanoid robot behaviors</p>
              </div>
            </div>
            <div className={styles.pathStep}>
              <div className={styles.stepNumber}>4</div>
              <div className={styles.stepContent}>
                <h3>Capstone Project</h3>
                <p>Create a complete AI-integrated humanoid robot system from scratch</p>
              </div>
            </div>
          </div>
        </section>
      </div>
      <ChatWidget />
    </Layout>
  );
}