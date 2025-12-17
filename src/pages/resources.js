import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Resources() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`Resources - ${siteConfig.title}`} description="Additional resources for Physical AI & Humanoid Robotics">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className="text--center">Learning Resources</h1>

            <h2>Official Documentation</h2>
            <ul>
              <li><a href="https://docs.ros.org/en/humble/" target="_blank" rel="noopener">ROS 2 Documentation</a></li>
              <li><a href="https://gazebosim.org/" target="_blank" rel="noopener">Gazebo Simulation</a></li>
              <li><a href="https://docs.omniverse.nvidia.com/isaacsim/latest/" target="_blank" rel="noopener">NVIDIA Isaac Sim</a></li>
              <li><a href="https://nvidia-isaac-ros.github.io/" target="_blank" rel="noopener">Isaac ROS</a></li>
            </ul>

            <h2>Research Papers</h2>
            <ul>
              <li><a href="https://arxiv.org/abs/2209.07434" target="_blank" rel="noopener">Embodied AI Survey</a></li>
              <li><a href="https://arxiv.org/abs/2306.00999" target="_blank" rel="noopener">VLA Systems Research</a></li>
              <li><a href="https://arxiv.org/abs/2302.12246" target="_blank" rel="noopener">Robotics and AI Integration</a></li>
              <li><a href="https://arxiv.org/abs/2209.03430" target="_blank" rel="noopener">Multimodal AI for Robotics</a></li>
            </ul>

            <h2>Tools & Libraries</h2>
            <ul>
              <li><a href="https://github.com/openai/whisper" target="_blank" rel="noopener">OpenAI Whisper</a></li>
              <li><a href="https://huggingface.co/docs/transformers/index" target="_blank" rel="noopener">Hugging Face Transformers</a></li>
              <li><a href="https://unity.com/solutions/robotics" target="_blank" rel="noopener">Unity Robotics</a></li>
              <li><a href="https://navigation.ros.org/" target="_blank" rel="noopener">ROS 2 Navigation</a></li>
            </ul>

            <h2>Community & Support</h2>
            <ul>
              <li><a href="https://robotics.stackexchange.com/" target="_blank" rel="noopener">Robotics Stack Exchange</a></li>
              <li><a href="https://discourse.ros.org/" target="_blank" rel="noopener">ROS Discourse</a></li>
              <li><a href="https://www.ros.org/" target="_blank" rel="noopener">ROS Official Website</a></li>
              <li><a href="https://developer.nvidia.com/isaac-ros-gems" target="_blank" rel="noopener">NVIDIA Robotics Developer Kit</a></li>
            </ul>

            <div className="margin-vert--lg">
              <h2>Additional Resources</h2>
              <p>For the most up-to-date resources and tools, please check the weekly breakdown sections in each module which contain regularly updated links and references.</p>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}