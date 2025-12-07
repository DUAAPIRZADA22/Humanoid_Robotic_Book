import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import Hero from '@site/src/components/Hero';
import FeaturesGrid from '@site/src/components/FeaturesGrid';
import LearningOutcomes from '@site/src/components/LearningOutcomes';
import Prerequisites from '@site/src/components/Prerequisites';
import ChaptersOverview from '@site/src/components/ChaptersOverview';
import CourseTimeline from '@site/src/components/CourseTimeline';
import Footer from '@site/src/components/Footer';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Hero
      title={siteConfig.title}
      subtitle={siteConfig.tagline}
      ctaText="Start Learning"
      ctaHref="/docs/intro"
    />
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} - ${siteConfig.tagline}`}
      description="Master embodied intelligence through comprehensive AI-driven robotics courses">
      <HomepageHeader />
      <main className={styles.homepageMain}>
        <FeaturesGrid />
        <LearningOutcomes />
        <Prerequisites />
        <ChaptersOverview />
        <CourseTimeline />
      </main>
      <Footer />
    </Layout>
  );
}