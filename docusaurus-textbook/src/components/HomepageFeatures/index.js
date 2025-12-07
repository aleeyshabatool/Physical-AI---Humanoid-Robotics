import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

// Physical AI and Humanoid Robotics Documentation Topics
const DocsTopics = [
  {
    title: 'Tutorial Intro',
    description: 'Get started with Physical AI and Humanoid Robotics - Overview and prerequisites',
    icon: '🎯',
    link: '/docs/intro',
    color: '#7c3aed', // Purple
  },
  {
    title: 'Introduction to Physical AI & Humanoid Robotics',
    description: 'Learn the fundamentals of Physical AI, robotics principles, and humanoid robot architecture',
    icon: '🤖',
    link: '/docs/chapter-1-introduction',
    color: '#2563eb', // Blue
  },
  {
    title: 'ROS 2 Fundamentals',
    description: 'Master ROS 2 basics, nodes, topics, services, and the ROS 2 ecosystem',
    icon: '🔄',
    link: '/docs/chapter-2-ros2-fundamentals',
    color: '#0d9488', // Teal
  },
  {
    title: 'Advanced ROS 2 and Tooling',
    description: 'Deep dive into advanced ROS 2 concepts, debugging tools, and best practices',
    icon: '⚙️',
    link: '/docs/chapter-3-advanced-ros2',
    color: '#db2777', // Pink
  },
  {
    title: 'Simulating Robots with Gazebo',
    description: 'Create and simulate robots using Gazebo simulator with ROS 2 integration',
    icon: '🌍',
    link: '/docs/chapter-4-simulating-robots-gazebo',
    color: '#ea580c', // Orange
  },
  {
    title: 'Introduction to NVIDIA Isaac and Omniverse',
    description: 'Explore NVIDIA Isaac platform and Omniverse for robotic simulation',
    icon: '🎮',
    link: '/docs/chapter-5-nvidia-isaac-omniverse',
    color: '#10b981', // Emerald
  },
  {
    title: 'Building & Simulating Robots in Isaac Sim',
    description: 'Practical guide to building and simulating robots using Isaac Sim',
    icon: '🏗️',
    link: '/docs/chapter-6-building-simulating-isaac-sim',
    color: '#8b5cf6', // Violet
  },
  {
    title: 'AI and Perception in Isaac Sim',
    description: 'Implement AI, computer vision, and perception systems in robotic simulations',
    icon: '👁️',
    link: '/docs/chapter-7-ai-perception-isaac-sim',
    color: '#f59e0b', // Amber
  },
];

function TopicCard({ icon, title, description, link, color }) {
  return (
    <div className={clsx('col col--4')}>
      <div 
        className={styles.topicCard}
        style={{
          '--card-accent': color,
        }}
      >
        <div className={styles.cardIcon}>
          <span className={styles.icon}>{icon}</span>
        </div>
        
        <div className={styles.cardContent}>
          <Heading as="h3" className={styles.cardTitle}>
            {title}
          </Heading>
          <p className={styles.cardDescription}>{description}</p>
          
          <Link
            className={styles.cardButton}
            to={link}
          >
            Read Chapter →
          </Link>
        </div>
        
        <div className={styles.cardDecoration}></div>
        
        {/* Progress indicator for chapters */}
        <div className={styles.chapterBadge}>
          {title.includes('Chapter') ? title.split(':')[0] : 'Intro'}
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            🤖 Physical AI & Humanoid Robotics
          </Heading>
          <p className={styles.sectionSubtitle}>
            Complete learning path from fundamentals to advanced AI-powered robotics simulation
          </p>
          
          {/* Learning Path Visual */}
          <div className={styles.learningPath}>
            <div className={styles.pathLine}></div>
            <div className={styles.pathSteps}>
              {DocsTopics.map((topic, idx) => (
                <div key={idx} className={styles.pathStep} style={{ '--step-color': topic.color }}>
                  <span className={styles.stepNumber}>{idx + 1}</span>
                </div>
              ))}
            </div>
          </div>
        </div>
        
        <div className="row">
          {DocsTopics.map((props, idx) => (
            <TopicCard key={idx} {...props} />
          ))}
        </div>
        
        <div className={styles.viewAllContainer}>
        
          
          <Link
            className={styles.viewAllButton}
            to="/docs/intro"
          >
            Start Learning Journey →
          </Link>
        </div>
      </div>
    </section>
  );
}