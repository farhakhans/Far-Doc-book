import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import RAGChatbot from '../components/RAGChatbot';
import styles from './rag-chatbot.module.css';

function RAGChatbotPage() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`Documentation RAG Chatbot - ${siteConfig.title}`}
      description="Retrieval-Augmented Generation (RAG) documentation assistant for Physical AI & Humanoid Robotics">
      <div className={styles.ragChatbotPage}>
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--12">
              <div className={styles.header}>
                <h1>
                  <span role="img" aria-label="robot">🤖</span> Documentation RAG Assistant
                </h1>
                <p className={styles.subtitle}>
                  I'm your <strong>Retrieval-Augmented Generation (RAG)</strong> assistant for the Physical AI & Humanoid Robotics documentation. I can search through the entire documentation to provide accurate, context-aware answers about ROS 2, Simulation Environments, NVIDIA Isaac, Vision-Language-Action Systems, and more.
                </p>
              </div>

              <div className={styles.chatContainer}>
                <RAGChatbot />
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default RAGChatbotPage;