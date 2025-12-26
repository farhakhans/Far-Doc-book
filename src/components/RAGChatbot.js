import React, { useState, useRef, useEffect } from 'react';
import './RAGChatbot.css';

const RAGChatbot = () => {
  // Load messages from localStorage to persist chat history
  const loadMessages = () => {
    const savedMessages = localStorage.getItem('ragChatMessages');
    if (savedMessages) {
      try {
        return JSON.parse(savedMessages);
      } catch (e) {
        console.error('Error parsing saved messages:', e);
        return [
          { id: 1, text: "Hello! I'm your RAG chatbot for the Physical AI Humanoid Robotics book. Ask me anything about robotics!", sender: 'bot' }
        ];
      }
    }
    return [
      { id: 1, text: "Hello! I'm your RAG chatbot for the Physical AI Humanoid Robotics book. Ask me anything about robotics!", sender: 'bot' }
    ];
  };

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState(loadMessages);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isTyping, setIsTyping] = useState(false);
  const [serverStatus, setServerStatus] = useState('checking'); // 'online', 'offline', 'checking'
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Save messages to localStorage whenever messages change
  useEffect(() => {
    localStorage.setItem('ragChatMessages', JSON.stringify(messages));
  }, [messages]);

  // Check server status periodically
  useEffect(() => {
    const checkInitialStatus = async () => {
      setServerStatus('checking');
      const status = await checkServerStatus();
      setServerStatus(status ? 'online' : 'offline');
    };

    checkInitialStatus();

    // Check server status every 30 seconds
    const interval = setInterval(async () => {
      const status = await checkServerStatus();
      setServerStatus(status ? 'online' : 'offline');
    }, 30000);

    return () => clearInterval(interval);
  }, []);

  // Add keyboard shortcut to toggle chat (Ctrl/Cmd + Shift + C)
  useEffect(() => {
    const handleKeyDown = (e) => {
      // Check if Ctrl/Cmd + Shift + C is pressed
      if ((e.ctrlKey || e.metaKey) && e.shiftKey && e.key.toLowerCase() === 'c') {
        e.preventDefault();
        toggleChat();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  }, [isOpen]);

  // Function to check if server is available
  const checkServerStatus = async () => {
    try {
      const response = await fetch('http://localhost:3001/api/health');
      return response.ok;
    } catch (error) {
      console.error('Server health check failed:', error);
      return false;
    }
  };

  const handleSend = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Check server status before sending
    const isServerAvailable = await checkServerStatus();
    if (!isServerAvailable) {
      const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
      setMessages(prev => [...prev, userMessage]);

      const serverErrorMessage = {
        id: Date.now() + 1,
        text: "The backend server is not running. Please start the server on port 3001 to use the chatbot.",
        sender: 'bot',
        isError: true,
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, serverErrorMessage]);
      return;
    }

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setIsTyping(true);

    try {
      // Send message to backend RAG API
      // Note: In development, we're calling the API server directly on port 3001
      // In production, this would need to be properly configured
      const response = await fetch('http://localhost:3001/api/rag/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: inputValue }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      if (data.success) {
        const botMessage = {
          id: Date.now() + 1,
          text: data.data.response,
          sender: 'bot',
          sources: data.data.sources || [],
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: data.error || "Sorry, I encountered an error processing your request. Please try again.",
          sender: 'bot',
          isError: true,
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Chat error:', error);

      // Check if it's a network error
      if (error.message.includes('fetch') || error.message.includes('network')) {
        const errorMessage = {
          id: Date.now() + 1,
          text: "It seems I can't connect to the server. Please make sure the backend server is running on port 3001.",
          sender: 'bot',
          isError: true,
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, errorMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: error.message || "Sorry, I'm having trouble connecting to the server. Please try again later.",
          sender: 'bot',
          isError: true,
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } finally {
      setIsLoading(false);
      setIsTyping(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend(e);
    }
  };

  // Auto-resize textarea as user types
  useEffect(() => {
    const textarea = inputRef.current;
    if (textarea) {
      textarea.style.height = 'auto';
      textarea.style.height = Math.min(textarea.scrollHeight, 100) + 'px';
    }
  }, [inputValue]);

  const clearChat = () => {
    setMessages([
      { id: 1, text: "Hello! I'm your RAG chatbot for the Physical AI Humanoid Robotics book. Ask me anything about robotics!", sender: 'bot' }
    ]);
    localStorage.removeItem('ragChatMessages');
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  // Function to format message text with basic markdown support
  const formatMessageText = (text) => {
    // Convert **text** to bold
    let formattedText = text.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');
    // Convert *text* to italics
    formattedText = formattedText.replace(/\*(.*?)\*/g, '<em>$1</em>');
    // Convert line breaks
    formattedText = formattedText.replace(/\n/g, '<br>');

    return { __html: formattedText };
  };

  return (
    <div className="rag-chatbot">
      <button
        className={`chatbot-toggle-button ${serverStatus === 'offline' ? 'server-offline' : ''} ${isOpen ? 'open-state' : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {messages.length > 1 ? (
          <span className="chat-unread-indicator">{messages.length - 1}</span>
        ) : null}
        {isOpen ? '❌' : (serverStatus === 'offline' ? '⚠️' : '🤖')}
      </button>

      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <div className="header-content">
              <div className="header-info">
                <h3>🤖 Robotics RAG Assistant</h3>
                <div className="server-status-indicator">
                  <div className={`status-dot ${serverStatus}`}></div>
                  <span className="status-text">
                    {serverStatus === 'online' ? 'Online' :
                     serverStatus === 'offline' ? 'Offline' : 'Checking...'}
                  </span>
                </div>
              </div>
              <div className="header-actions">
                <button
                  className="clear-chat-button"
                  onClick={clearChat}
                  aria-label="Clear chat history"
                  title="Clear chat history"
                >
                  🗑️
                </button>
              </div>
            </div>
          </div>
          <div className="chatbot-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.sender}-message ${message.isError ? 'error-message' : ''}`}
              >
                <div
                  className="message-text"
                  dangerouslySetInnerHTML={formatMessageText(message.text)}
                />
                {message.sources && message.sources.length > 0 && (
                  <div className="message-sources">
                    <strong>Sources:</strong>
                    <ul>
                      {message.sources.map((source, index) => (
                        <li key={index}>
                          <a
                            href={source.url}
                            target="_blank"
                            rel="noopener noreferrer"
                            aria-label={`Source: ${source.title}`}
                          >
                            {source.title}
                          </a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
            {(isLoading || isTyping) && (
              <div className="message bot-message typing-message">
                <div className="message-text">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form className="chatbot-input-form" onSubmit={handleSend}>
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask about robotics..."
              disabled={isLoading}
              aria-label="Type your message"
              rows="1"
              className="chat-input-textarea"
            />
            <button
              type="submit"
              disabled={isLoading}
              aria-label="Send message"
              className="send-button"
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </form>
          <div className="chatbot-footer">
            <small>Press Enter to send, Shift+Enter for new line</small>
            <small className="keyboard-shortcut">Tip: Use Ctrl+Shift+C to toggle chat</small>
          </div>
        </div>
      )}
    </div>
  );
};

export default RAGChatbot;