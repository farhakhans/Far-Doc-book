import React, { useState, useEffect, useRef } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const RAGChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:5003/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          contextSize: 5  // Increased context size for better answers
        })
      });

      const data = await response.json();

      if (response.ok) {
        // Process the response to extract more specific information
        const processedResponse = processResponse(data);

        const aiMessage = {
          id: Date.now() + 1,
          text: processedResponse,
          sender: 'ai',
          contextSources: data.contextSources,
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, aiMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: `Error: ${data.error || 'Failed to get response'}`,
          sender: 'ai',
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: `Connection error: ${error.message}`,
        sender: 'ai',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to process the response and extract specific information
  const processResponse = (data) => {
    // If we have context sources, try to extract more specific information
    if (data.contextSources && data.contextSources.length > 0) {
      // Create a more structured response focusing on the most relevant source
      const mostRelevantSource = data.contextSources[0]; // Most relevant based on similarity score

      // Extract the core information from the response and focus on the specific answer
      let processedText = data.response;

      // If the response is too general, try to focus on the most relevant document
      if (data.response.includes("Based on the documentation")) {
        // Extract more specific information from the context
        const contextParts = data.response.split('---');
        if (contextParts.length > 1) {
          // Get the most relevant part based on the first context source
          const specificInfo = `Based on "${mostRelevantSource.title}":\n\n${contextParts[0].replace(/File:.*\nTitle:.*\nContent:/, '').trim()}`;

          if (specificInfo.trim() !== "") {
            processedText = `${specificInfo}\n\nFor more details, see: ${mostRelevantSource.filepath}`;
          }
        }
      }

      return processedText;
    }

    return data.response;
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const quickQuestions = [
    "What is ROS 2?",
    "Tell me about simulation environments",
    "Explain NVIDIA Isaac platform",
    "What are Vision-Language-Action systems?",
    "Hardware requirements",
    "Weekly breakdown"
  ];

  return (
    <BrowserOnly>
      {() => {
        const [isMounted, setIsMounted] = useState(false);

        useEffect(() => {
          setIsMounted(true);
        }, []);

        if (!isMounted) {
          return <div className="rag-chatbot-placeholder">Loading RAG Chatbot...</div>;
        }

        return (
          <div className={`rag-chatbot ${isOpen ? 'rag-chatbot--open' : ''}`}>
            {isOpen ? (
              <div className="rag-chatbot__container">
                <div className="rag-chatbot__header">
                  <div className="rag-chatbot__title">
                    <span role="img" aria-label="robot">🤖</span>
                    <span>Documentation RAG Assistant</span>
                  </div>
                  <button
                    className="rag-chatbot__close"
                    onClick={() => setIsOpen(false)}
                    aria-label="Close chat"
                  >
                    ×
                  </button>
                </div>

                <div className="rag-chatbot__messages">
                  {messages.length === 0 ? (
                    <div className="rag-chatbot__welcome">
                      <h3>Hello! 👋</h3>
                      <p>I'm your <strong>Retrieval-Augmented Generation (RAG)</strong> assistant for the Physical AI & Humanoid Robotics documentation.</p>
                      <p>Ask me anything about the documentation, and I'll search through it to provide accurate answers.</p>
                    </div>
                  ) : (
                    messages.map((message) => (
                      <div
                        key={message.id}
                        className={`rag-chatbot__message rag-chatbot__message--${message.sender}`}
                      >
                        <div className="rag-chatbot__message-header">
                          <div className={`rag-chatbot__message-icon rag-chatbot__message-icon--${message.sender}`}>
                            {message.sender === 'user' ? (
                              <span role="img" aria-label="user">👤</span>
                            ) : (
                              <span role="img" aria-label="robot">🤖</span>
                            )}
                          </div>
                          <span className="rag-chatbot__message-sender">
                            {message.sender === 'user' ? 'You' : 'RAG Assistant'}
                          </span>
                        </div>
                        <div className="rag-chatbot__message-content">
                          {message.text.split('\n').map((line, i) => (
                            <div key={i}>{line}</div>
                          ))}
                        </div>
                        {message.contextSources && message.contextSources.length > 0 && (
                          <div className="rag-chatbot__message-context">
                            <strong>Referenced documents:</strong>
                            {message.contextSources.map((source, idx) => (
                              <div key={idx} className="rag-chatbot__context-source">
                                <em>{source.title}</em> ({source.filepath}) - Relevance: {(source.similarity * 100).toFixed(1)}%
                              </div>
                            ))}
                          </div>
                        )}
                      </div>
                    ))
                  )}
                  {isLoading && (
                    <div className="rag-chatbot__message rag-chatbot__message--ai">
                      <div className="rag-chatbot__message-header">
                        <div className="rag-chatbot__message-icon rag-chatbot__message-icon--ai">
                          <span role="img" aria-label="robot">🤖</span>
                        </div>
                        <span className="rag-chatbot__message-sender">RAG Assistant</span>
                      </div>
                      <div className="rag-chatbot__message-content">
                        <div className="rag-chatbot__typing-indicator">
                          <div className="rag-chatbot__typing-dot"></div>
                          <div className="rag-chatbot__typing-dot"></div>
                          <div className="rag-chatbot__typing-dot"></div>
                        </div>
                        <div className="rag-chatbot__status">Searching documentation...</div>
                      </div>
                    </div>
                  )}
                  <div ref={messagesEndRef} />
                </div>

                <div className="rag-chatbot__input-area">
                  <textarea
                    id="rag-chatbot-input"
                    className="rag-chatbot__input"
                    value={inputValue}
                    onChange={(e) => setInputValue(e.target.value)}
                    onKeyPress={handleKeyPress}
                    placeholder="Ask anything about Physical AI & Humanoid Robotics documentation..."
                    rows="1"
                    disabled={isLoading}
                  />
                  <button
                    className="rag-chatbot__send-button"
                    onClick={sendMessage}
                    disabled={isLoading || !inputValue.trim()}
                  >
                    <span role="img" aria-label="send">📤</span>
                  </button>
                </div>
              </div>
            ) : null}

            <button
              className="rag-chatbot__trigger"
              onClick={() => setIsOpen(!isOpen)}
              aria-label="Open RAG chatbot"
            >
              <span role="img" aria-label="robot">🤖</span>
            </button>
          </div>
        );
      }}
    </BrowserOnly>
  );
};

export default RAGChatbot;