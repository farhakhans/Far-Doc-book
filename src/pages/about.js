import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function About() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`About ${siteConfig.title}`} description="About the Physical AI & Humanoid Robotics Book">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className="text--center">About This Book</h1>
            <p>
              The <strong>{siteConfig.title}</strong> is a comprehensive educational resource designed to bridge the gap
              between digital artificial intelligence and physical robot control.
            </p>

            <h2>Our Mission</h2>
            <p>
              This book aims to provide students and engineers with the knowledge and practical skills needed to implement
              AI algorithms that control physical systems. We focus on embodied intelligence - the idea that intelligence
              emerges from the interaction between an agent and its environment.
            </p>

            <h2>Target Audience</h2>
            <p>
              This book is designed for:
            </p>
            <ul>
              <li>Undergraduate and graduate students in robotics</li>
              <li>Researchers in embodied AI</li>
              <li>Engineers working with humanoid robots</li>
              <li>Anyone interested in the intersection of AI and physical systems</li>
            </ul>

            <h2>Technologies Covered</h2>
            <p>
              The book covers modern robotics tools and frameworks including:
            </p>
            <ul>
              <li>ROS 2 (Robot Operating System 2)</li>
              <li>Gazebo and Unity simulation environments</li>
              <li>NVIDIA Isaac platform</li>
              <li>Vision-Language-Action (VLA) systems</li>
            </ul>

            <div className="text--center margin-vert--lg">
              <a className="button button--primary button--lg" href="/docs/intro">
                Start Learning
              </a>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}