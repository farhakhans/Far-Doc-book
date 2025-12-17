import React, { useState, useEffect, useRef } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const ChatBot = () => {
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
      const response = await fetch('http://localhost:5000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          contextSize: 5
        })
      });

      const data = await response.json();

      if (response.ok) {
        const aiMessage = {
          id: Date.now() + 1,
          text: data.response,
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

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <BrowserOnly>
      {() => {
        const [isMounted, setIsMounted] = useState(false);

        useEffect(() => {
          setIsMounted(true);
        }, []);

        if (!isMounted) {
          return <div className="chatbot-placeholder">Loading chatbot...</div>;
        }

        return (
          <div className={`chatbot ${isOpen ? 'chatbot--open' : ''}`}>
            {isOpen ? (
              <div className="chatbot__container">
                <div className="chatbot__header">
                  <div className="chatbot__title">
                    <span role="img" aria-label="robot">🤖</span>
                    <span>Documentation Assistant</span>
                  </div>
                  <button
                    className="chatbot__close"
                    onClick={() => setIsOpen(false)}
                    aria-label="Close chat"
                  >
                    ×
                  </button>
                </div>

                <div className="chatbot__messages">
                  {messages.length === 0 ? (
                    <div className="chatbot__welcome">
                      <h3>Hello! 👋</h3>
                      <p>Ask me anything about Physical AI & Humanoid Robotics documentation.</p>
                    </div>
                  ) : (
                    messages.map((message) => (
                      <div
                        key={message.id}
                        className={`chatbot__message chatbot__message--${message.sender}`}
                      >
                        <div className="chatbot__message-header">
                          <div className={`chatbot__message-icon chatbot__message-icon--${message.sender}`}>
                            {message.sender === 'user' ? (
                              <span role="img" aria-label="user">👤</span>
                            ) : (
                              <span role="img" aria-label="robot">🤖</span>
                            )}
                          </div>
                          <span className="chatbot__message-sender">
                            {message.sender === 'user' ? 'You' : 'Assistant'}
                          </span>
                        </div>
                        <div className="chatbot__message-content">
                          {message.text.split('\n').map((line, i) => (
                            <div key={i}>{line}</div>
                          ))}
                        </div>
                        {message.contextSources && message.contextSources.length > 0 && (
                          <div className="chatbot__message-context">
                            <strong>References:</strong>
                            {message.contextSources.map((source, idx) => (
                              <div key={idx} className="chatbot__context-source">
                                <em>{source.title}</em> ({source.filepath}) - {(source.similarity * 100).toFixed(1)}%
                              </div>
                            ))}
                          </div>
                        )}
                      </div>
                    ))
                  )}
                  {isLoading && (
                    <div className="chatbot__message chatbot__message--ai">
                      <div className="chatbot__message-header">
                        <div className="chatbot__message-icon chatbot__message-icon--ai">
                          <span role="img" aria-label="robot">🤖</span>
                        </div>
                        <span className="chatbot__message-sender">Assistant</span>
                      </div>
                      <div className="chatbot__message-content">
                        <div className="chatbot__typing-indicator">
                          <div className="chatbot__typing-dot"></div>
                          <div className="chatbot__typing-dot"></div>
                          <div className="chatbot__typing-dot"></div>
                        </div>
                        <div className="chatbot__status">Searching documentation...</div>
                      </div>
                    </div>
                  )}
                  <div ref={messagesEndRef} />
                </div>

                <div className="chatbot__input-area">
                  <textarea
                    id="chatbot-input"
                    className="chatbot__input"
                    value={inputValue}
                    onChange={(e) => setInputValue(e.target.value)}
                    onKeyPress={handleKeyPress}
                    placeholder="Ask about ROS 2, simulation, NVIDIA Isaac, VLA systems..."
                    rows="1"
                    disabled={isLoading}
                  />
                  <button
                    className="chatbot__send-button"
                    onClick={sendMessage}
                    disabled={isLoading || !inputValue.trim()}
                  >
                    <span role="img" aria-label="send">📤</span>
                  </button>
                </div>
              </div>
            ) : null}

            <button
              className="chatbot__trigger"
              onClick={() => setIsOpen(!isOpen)}
              aria-label="Open chatbot"
            >
              <span role="img" aria-label="robot">🤖</span>
            </button>
          </div>
        );
      }}
    </BrowserOnly>
  );
};

export default ChatBot;