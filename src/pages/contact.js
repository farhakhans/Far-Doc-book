import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Contact() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`Contact - ${siteConfig.title}`} description="Contact information for Physical AI & Humanoid Robotics Book">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className="text--center">Contact & Support</h1>

            <h2>Get In Touch</h2>
            <p>
              If you have questions about the Physical AI & Humanoid Robotics content, need technical support,
              or want to contribute to the project, please reach out to us.
            </p>

            <h2>Support Channels</h2>
            <div className="margin--md">
              <h3>Documentation Issues</h3>
              <p>
                For issues with the book content, documentation errors, or suggestions for improvements,
                please open an issue on our <a href="https://github.com/your-username/physical-ai-humanoid-robotics-book" target="_blank" rel="noopener">GitHub repository</a>.
              </p>
            </div>

            <div className="margin--md">
              <h3>Technical Questions</h3>
              <p>
                For technical questions about ROS 2, simulation environments, or implementation details,
                consider these community resources:
              </p>
              <ul>
                <li><a href="https://answers.ros.org/questions/" target="_blank" rel="noopener">ROS Answers</a></li>
                <li><a href="https://discourse.ros.org/" target="_blank" rel="noopener">ROS Discourse</a></li>
                <li><a href="https://robotics.stackexchange.com/" target="_blank" rel="noopener">Robotics Stack Exchange</a></li>
              </ul>
            </div>

            <div className="margin--md">
              <h3>Project Contributions</h3>
              <p>
                We welcome contributions to the Physical AI & Humanoid Robotics book project!
                Whether it's fixing typos, adding new content, or improving examples,
                your contributions help improve the learning experience for everyone.
              </p>
              <p>
                To contribute, please visit our <a href="https://github.com/your-username/physical-ai-humanoid-robotics-book" target="_blank" rel="noopener">GitHub repository</a>
                and follow our contribution guidelines.
              </p>
            </div>

            <div className="margin--md">
              <h3>Academic Inquiries</h3>
              <p>
                For academic use of this material, adoption in courses, or educational partnerships,
                please contact us directly through the GitHub repository issues or your academic institution's
                appropriate channels.
              </p>
            </div>

            <div className="text--center margin-vert--lg">
              <a className="button button--primary button--lg" href="/docs/intro">
                Back to Learning
              </a>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}